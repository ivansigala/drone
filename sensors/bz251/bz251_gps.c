/*
 * bz251_gps.c
 *
 *  Created on : Mar 12, 2026
 *  Updated    : May 12, 2026 — full UBX rewrite.
 *
 *  Driver for the Beitian BZ251 (u-blox M-class) GNSS module.
 *
 *  Why UBX instead of NMEA:
 *    - NMEA at 10 Hz is ~280 chars/fix and ASCII-parsed; UBX NAV-PVT is one
 *      100-byte binary frame that gives us *everything we need in a single
 *      message*: position, altitude, NED velocity, ground speed, heading,
 *      fix info and timestamp. No sscanf, no DDMM.MMMM conversion, no waiting
 *      for multiple sentences to reconstruct a state.
 *    - At 115200 baud, 10 Hz NAV-PVT consumes ~9% of the link instead of the
 *      ~24% the default NMEA bundle eats.
 *
 *  Boot sequence performed by the task:
 *      1. Bring up LPUART1 + EDMA RX (TX uses blocking polled writes — see
 *         note further down on why EDMA TX is overkill for tiny CFG frames).
 *      2. Mute all standard NMEA sentences on the current port.
 *      3. Set the navigation rate to 10 Hz (UBX-CFG-RATE measRate = 100 ms).
 *      4. Enable UBX-NAV-PVT at every fix (UBX-CFG-MSG).
 *      5. Enter a forever loop: DMA-read a chunk, feed bytes through the
 *         UBX parser state machine, update global_gps_data when a valid
 *         NAV-PVT arrives.
 *
 *  IMPORTANT — module baud rate:
 *      This driver talks to the module at 115200. u-blox modules typically
 *      *ship* at 9600 baud with NMEA enabled. If your BZ251 is still at
 *      factory defaults, run u-center once to set 115200 + UBX and either
 *      save with UBX-CFG-CFG, or extend ubx_configure_gps() below to send
 *      UBX-CFG-PRT at 9600 first and re-init the LPUART. The hooks are
 *      noted in the comments where they belong.
 */

#include "bz251_gps.h"
#include "uart_driver_mcxn947.h"
#include "fsl_debug_console.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* ---------------------------------------------------------------------------
 * UBX protocol constants
 * ------------------------------------------------------------------------- */
#define UBX_SYNC_1              0xB5
#define UBX_SYNC_2              0x62

#define UBX_CLASS_NAV           0x01
#define UBX_CLASS_ACK           0x05
#define UBX_CLASS_CFG           0x06
#define UBX_CLASS_NMEA          0xF0

#define UBX_ID_NAV_PVT          0x07

#define UBX_ID_CFG_PRT          0x00
#define UBX_ID_CFG_MSG          0x01
#define UBX_ID_CFG_RATE         0x08
#define UBX_ID_CFG_CFG          0x09

/* NMEA standard message IDs (under class 0xF0) */
#define NMEA_ID_GGA             0x00
#define NMEA_ID_GLL             0x01
#define NMEA_ID_GSA             0x02
#define NMEA_ID_GSV             0x03
#define NMEA_ID_RMC             0x04
#define NMEA_ID_VTG             0x05
#define NMEA_ID_ZDA             0x08

/* NAV-PVT payload layout (offsets within the 92-byte payload).
 * Reference: u-blox protocol spec, section "UBX-NAV-PVT (0x01 0x07)". */
#define PVT_OFF_ITOW            0    /* U4  ms */
#define PVT_OFF_YEAR            4    /* U2  */
#define PVT_OFF_MONTH           6    /* U1  */
#define PVT_OFF_DAY             7    /* U1  */
#define PVT_OFF_HOUR            8    /* U1  */
#define PVT_OFF_MIN             9    /* U1  */
#define PVT_OFF_SEC             10   /* U1  */
#define PVT_OFF_FIXTYPE         20   /* U1  */
#define PVT_OFF_FLAGS           21   /* X1  */
#define PVT_OFF_NUMSV           23   /* U1  */
#define PVT_OFF_LON             24   /* I4  1e-7 deg */
#define PVT_OFF_LAT             28   /* I4  1e-7 deg */
#define PVT_OFF_HEIGHT          32   /* I4  mm above ellipsoid */
#define PVT_OFF_HMSL            36   /* I4  mm above mean sea level */
#define PVT_OFF_VELN            48   /* I4  mm/s North */
#define PVT_OFF_VELE            52   /* I4  mm/s East */
#define PVT_OFF_VELD            56   /* I4  mm/s Down */
#define PVT_OFF_GSPEED          60   /* I4  mm/s ground speed (2-D) */
#define PVT_OFF_HEADMOT         64   /* I4  1e-5 deg, heading of motion */

#define PVT_PAYLOAD_LEN         92

/* 1 m/s = 1.9438444924 international knots (1852 m / 3600 s exact). */
#define MPS_TO_KNOTS            1.9438444924f

/* DMA chunk size — large enough to hold ~1 NAV-PVT frame (100 B wire),
 * small enough that EDMA completion fires every 100 ms at 115200 baud,
 * giving us a fresh fix at the native 10 Hz update rate. */
#define DMA_CHUNK_SIZE          128

/* ---------------------------------------------------------------------------
 * Module-private state
 * ------------------------------------------------------------------------- */
gps_data_t              global_gps_data;
SemaphoreHandle_t       gps_data_mutex;

static SemaphoreHandle_t s_dma_rx_sem;
static uart_ctrl_t       s_gps_uart;
static uint8_t           s_dma_rx_buffer[DMA_CHUNK_SIZE];

/* ---------------------------------------------------------------------------
 * UBX parser — byte-at-a-time state machine.
 *
 * Keeping state across calls lets us cope with the DMA boundary cutting a
 * frame in half: the next chunk picks up exactly where the previous one
 * stopped. Recovery from desync is automatic — any byte that isn't 0xB5 at
 * the start drops us back to SYNC1.
 * ------------------------------------------------------------------------- */
typedef enum {
    UBX_S_SYNC1,
    UBX_S_SYNC2,
    UBX_S_CLASS,
    UBX_S_ID,
    UBX_S_LEN_L,
    UBX_S_LEN_H,
    UBX_S_PAYLOAD,
    UBX_S_CK_A,
    UBX_S_CK_B,
} ubx_state_t;

typedef struct {
    ubx_state_t state;
    uint8_t     msg_class;
    uint8_t     msg_id;
    uint16_t    plen;
    uint16_t    pidx;
    uint8_t     payload[PVT_PAYLOAD_LEN];
    uint8_t     ck_a, ck_b;            /* checksum bytes pulled off the wire   */
    uint8_t     calc_a, calc_b;        /* checksum running total we compute    */
} ubx_parser_t;

static ubx_parser_t s_parser;

/* ---------------------------------------------------------------------------
 * Little-endian extractors. The payload comes out of the wire LE regardless
 * of host endianness — do not cast through pointers, that's UB on Cortex-M
 * when the address isn't naturally aligned (NAV-PVT has plenty of odd-offset
 * I4 fields).
 * ------------------------------------------------------------------------- */
static inline int32_t  rd_i4(const uint8_t *b, int o) {
    return (int32_t)((uint32_t)b[o]
                   | ((uint32_t)b[o + 1] << 8)
                   | ((uint32_t)b[o + 2] << 16)
                   | ((uint32_t)b[o + 3] << 24));
}
static inline uint32_t rd_u4(const uint8_t *b, int o) {
    return  (uint32_t)b[o]
         | ((uint32_t)b[o + 1] << 8)
         | ((uint32_t)b[o + 2] << 16)
         | ((uint32_t)b[o + 3] << 24);
}
static inline uint16_t rd_u2(const uint8_t *b, int o) {
    return (uint16_t)b[o] | ((uint16_t)b[o + 1] << 8);
}

/* ---------------------------------------------------------------------------
 * UBX 8-bit Fletcher checksum over (class | id | len_lo | len_hi | payload).
 * ------------------------------------------------------------------------- */
static void ubx_checksum(const uint8_t *buf, uint16_t len,
                         uint8_t *ck_a_out, uint8_t *ck_b_out)
{
    uint8_t a = 0, b = 0;
    for (uint16_t i = 0; i < len; ++i) {
        a += buf[i];
        b += a;
    }
    *ck_a_out = a;
    *ck_b_out = b;
}

/* ---------------------------------------------------------------------------
 * Build a UBX frame in a scratch buffer and push it out the LPUART.
 *
 * Frame layout: sync1 sync2 class id len_lo len_hi <payload...> ck_a ck_b
 * Configuration payloads we send are tiny (≤ 16 bytes) — the static frame
 * buffer is sized accordingly. Blocking write because we only call this a
 * handful of times at boot; we want each command actually on the wire
 * before we issue the next.
 * ------------------------------------------------------------------------- */
static void ubx_send(uint8_t cls, uint8_t id,
                     const uint8_t *payload, uint16_t plen)
{
    static uint8_t frame[8 + 32];
    if (plen > sizeof(frame) - 8) {
        return;
    }

    frame[0] = UBX_SYNC_1;
    frame[1] = UBX_SYNC_2;
    frame[2] = cls;
    frame[3] = id;
    frame[4] = (uint8_t)(plen & 0xFF);
    frame[5] = (uint8_t)(plen >> 8);
    if (payload && plen) {
        memcpy(&frame[6], payload, plen);
    }

    uint8_t a, b;
    ubx_checksum(&frame[2], (uint16_t)(4 + plen), &a, &b);
    frame[6 + plen] = a;
    frame[7 + plen] = b;

    uart_write_bytes_blocking(&s_gps_uart, frame, (uint32_t)(8 + plen));
}

/* ---------------------------------------------------------------------------
 * Configuration helpers — each builds a small UBX-CFG-* payload and ships it.
 * ------------------------------------------------------------------------- */

/* UBX-CFG-MSG (short form, 3 B): set the output rate of a given message on
 * the *current* IO port. rate=0 disables, rate=1 means "send once per fix". */
static void ubx_disable_nmea(uint8_t nmea_id)
{
    uint8_t pl[3] = { UBX_CLASS_NMEA, nmea_id, 0x00 };
    ubx_send(UBX_CLASS_CFG, UBX_ID_CFG_MSG, pl, sizeof(pl));
}

static void ubx_enable_nav_pvt(void)
{
    uint8_t pl[3] = { UBX_CLASS_NAV, UBX_ID_NAV_PVT, 0x01 };
    ubx_send(UBX_CLASS_CFG, UBX_ID_CFG_MSG, pl, sizeof(pl));
}

/* UBX-CFG-RATE (6 B): measRate = ms between measurements,
 *                     navRate  = how many measurements per nav solution (1),
 *                     timeRef  = 1 → align solutions to GPS time. */
static void ubx_set_rate_10hz(void)
{
    uint8_t pl[6];
    uint16_t meas_rate_ms = 100;   /* 10 Hz */
    uint16_t nav_rate     = 1;
    uint16_t time_ref     = 1;     /* 0 = UTC, 1 = GPS */
    pl[0] = (uint8_t)(meas_rate_ms & 0xFF);
    pl[1] = (uint8_t)(meas_rate_ms >> 8);
    pl[2] = (uint8_t)(nav_rate     & 0xFF);
    pl[3] = (uint8_t)(nav_rate     >> 8);
    pl[4] = (uint8_t)(time_ref     & 0xFF);
    pl[5] = (uint8_t)(time_ref     >> 8);
    ubx_send(UBX_CLASS_CFG, UBX_ID_CFG_RATE, pl, sizeof(pl));
}

/* UBX-CFG-CFG (13 B): persist the current config to BBR + Flash so we don't
 * have to send the whole sequence on every cold boot. Left available but
 * NOT called by default — keeps boots deterministic and avoids wearing out
 * the module's flash if you're iterating. Uncomment the call in
 * ubx_configure_gps() once your settings are stable. */
static void ubx_save_config(void)
{
    uint8_t pl[13] = { 0 };
    /* saveMask = 0x0000061F → all sections (ioPort, msgConf, infMsg,
     *                                       navConf, rxmConf, antConf) */
    pl[4]  = 0x1F;
    pl[5]  = 0x06;
    /* deviceMask: BBR (bit0) | Flash (bit1) | I2C-EEPROM (bit4) */
    pl[12] = 0x17;
    ubx_send(UBX_CLASS_CFG, UBX_ID_CFG_CFG, pl, sizeof(pl));
}

/* ---------------------------------------------------------------------------
 * Boot-time configuration sequence. Small delays between commands give the
 * module time to apply each one and avoid back-pressure on its RX buffer.
 *
 * NOTE on baud-rate switching: if your BZ251 is still at 9600 (factory
 * default), do this *before* the rest of the sequence:
 *
 *      1. uart_init() the LPUART at 9600.
 *      2. Send a UBX-CFG-PRT for UART1 setting baudRate=115200.
 *      3. Tear down LPUART, re-init at 115200, continue with the sequence
 *         below.
 *
 * We don't do that here because the existing wiring already runs at 115200
 * — the original driver was reading $GNGGA at 115200 successfully, which
 * means the module is already configured for it.
 * ------------------------------------------------------------------------- */
static void ubx_configure_gps(void)
{
    /* Let the module finish its own boot. u-blox boot is < 100 ms but the
     * antenna housekeeping after first power-up is closer to 200 ms. */
    vTaskDelay(pdMS_TO_TICKS(250));

    /* Mute every standard NMEA sentence so the RX stream is pure binary. */
    ubx_disable_nmea(NMEA_ID_GGA); vTaskDelay(pdMS_TO_TICKS(20));
    ubx_disable_nmea(NMEA_ID_GLL); vTaskDelay(pdMS_TO_TICKS(20));
    ubx_disable_nmea(NMEA_ID_GSA); vTaskDelay(pdMS_TO_TICKS(20));
    ubx_disable_nmea(NMEA_ID_GSV); vTaskDelay(pdMS_TO_TICKS(20));
    ubx_disable_nmea(NMEA_ID_RMC); vTaskDelay(pdMS_TO_TICKS(20));
    ubx_disable_nmea(NMEA_ID_VTG); vTaskDelay(pdMS_TO_TICKS(20));
    ubx_disable_nmea(NMEA_ID_ZDA); vTaskDelay(pdMS_TO_TICKS(20));

    /* Bump the nav engine to 10 Hz. */
    ubx_set_rate_10hz();           vTaskDelay(pdMS_TO_TICKS(20));

    /* Enable the only message we actually want. */
    ubx_enable_nav_pvt();          vTaskDelay(pdMS_TO_TICKS(20));

    /* Persist to flash once your config is stable: */
    /* ubx_save_config(); */
    (void)ubx_save_config;   /* silence "defined but not used" until enabled */
}

/* ---------------------------------------------------------------------------
 * EDMA-RX callback — fires when the chunk has been filled. Hands off to the
 * task via a binary semaphore so the parser runs at task context, not ISR.
 * ------------------------------------------------------------------------- */
static void gps_dma_callback(LPUART_Type *base, lpuart_edma_handle_t *handle,
                             status_t status, void *userData)
{
    (void)base; (void)handle; (void)status; (void)userData;
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(s_dma_rx_sem, &hpw);
    portYIELD_FROM_ISR(hpw);
}

/* ---------------------------------------------------------------------------
 * Decode one NAV-PVT payload into a local struct, then publish it under
 * the mutex. We copy locally first so the mutex hold is < 1 µs.
 * ------------------------------------------------------------------------- */
static void handle_nav_pvt(const uint8_t *p)
{
    gps_data_t out;
    memset(&out, 0, sizeof(out));

    /* Timestamp */
    out.i_tow_ms       = rd_u4(p, PVT_OFF_ITOW);
    out.year           = rd_u2(p, PVT_OFF_YEAR);
    out.month          = p[PVT_OFF_MONTH];
    out.day            = p[PVT_OFF_DAY];
    out.hour           = p[PVT_OFF_HOUR];
    out.min            = p[PVT_OFF_MIN];
    out.sec            = p[PVT_OFF_SEC];

    /* Fix info */
    out.fix_type       = p[PVT_OFF_FIXTYPE];
    out.satellites     = p[PVT_OFF_NUMSV];

    /* Position */
    int32_t lon_i      = rd_i4(p, PVT_OFF_LON);
    int32_t lat_i      = rd_i4(p, PVT_OFF_LAT);
    int32_t height_mm  = rd_i4(p, PVT_OFF_HEIGHT);
    int32_t hmsl_mm    = rd_i4(p, PVT_OFF_HMSL);

    out.longitude_deg  = (double)lon_i * 1e-7;
    out.latitude_deg   = (double)lat_i * 1e-7;
    out.altitude_ell_m = (float)height_mm / 1000.0f;
    out.altitude_msl_m = (float)hmsl_mm   / 1000.0f;

    /* Velocity */
    int32_t velN_mm    = rd_i4(p, PVT_OFF_VELN);
    int32_t velE_mm    = rd_i4(p, PVT_OFF_VELE);
    int32_t velD_mm    = rd_i4(p, PVT_OFF_VELD);
    int32_t gspd_mm    = rd_i4(p, PVT_OFF_GSPEED);
    int32_t head_e5    = rd_i4(p, PVT_OFF_HEADMOT);

    out.vel_north_mps      = (float)velN_mm / 1000.0f;
    out.vel_east_mps       = (float)velE_mm / 1000.0f;
    out.vel_down_mps       = (float)velD_mm / 1000.0f;
    out.ground_speed_mps   = (float)gspd_mm / 1000.0f;
    out.ground_speed_knots = out.ground_speed_mps * MPS_TO_KNOTS;
    out.heading_deg        = (float)head_e5 * 1e-5f;

    /* fixType >= 2 means we have at least a 2-D fix. */
    out.is_valid = (out.fix_type >= 2);

    if (xSemaphoreTake(gps_data_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        global_gps_data = out;
        xSemaphoreGive(gps_data_mutex);
    }
}

/* ---------------------------------------------------------------------------
 * Feed one byte through the UBX state machine. Verifies the checksum at
 * the end of every frame before dispatching.
 * ------------------------------------------------------------------------- */
static void ubx_parser_feed(ubx_parser_t *pr, uint8_t b)
{
    switch (pr->state) {

    case UBX_S_SYNC1:
        if (b == UBX_SYNC_1) {
            pr->state = UBX_S_SYNC2;
        }
        break;

    case UBX_S_SYNC2:
        if (b == UBX_SYNC_2) {
            pr->state  = UBX_S_CLASS;
            pr->calc_a = 0;
            pr->calc_b = 0;
        } else if (b == UBX_SYNC_1) {
            /* Stay parked on SYNC2 — handles a run of 0xB5 bytes correctly. */
        } else {
            pr->state = UBX_S_SYNC1;
        }
        break;

    case UBX_S_CLASS:
        pr->msg_class = b;
        pr->calc_a   += b;
        pr->calc_b   += pr->calc_a;
        pr->state     = UBX_S_ID;
        break;

    case UBX_S_ID:
        pr->msg_id  = b;
        pr->calc_a += b;
        pr->calc_b += pr->calc_a;
        pr->state   = UBX_S_LEN_L;
        break;

    case UBX_S_LEN_L:
        pr->plen    = b;
        pr->calc_a += b;
        pr->calc_b += pr->calc_a;
        pr->state   = UBX_S_LEN_H;
        break;

    case UBX_S_LEN_H:
        pr->plen   |= ((uint16_t)b << 8);
        pr->calc_a += b;
        pr->calc_b += pr->calc_a;
        pr->pidx    = 0;
        if (pr->plen > sizeof(pr->payload)) {
            /* Bigger than anything we care about — drop it and resync. */
            pr->state = UBX_S_SYNC1;
        } else if (pr->plen == 0) {
            pr->state = UBX_S_CK_A;
        } else {
            pr->state = UBX_S_PAYLOAD;
        }
        break;

    case UBX_S_PAYLOAD:
        pr->payload[pr->pidx++] = b;
        pr->calc_a += b;
        pr->calc_b += pr->calc_a;
        if (pr->pidx >= pr->plen) {
            pr->state = UBX_S_CK_A;
        }
        break;

    case UBX_S_CK_A:
        pr->ck_a  = b;
        pr->state = UBX_S_CK_B;
        break;

    case UBX_S_CK_B:
        pr->ck_b  = b;
        pr->state = UBX_S_SYNC1;
        if (pr->ck_a == pr->calc_a && pr->ck_b == pr->calc_b) {
            /* Valid frame — dispatch the ones we care about. */
            if (pr->msg_class == UBX_CLASS_NAV &&
                pr->msg_id    == UBX_ID_NAV_PVT &&
                pr->plen      == PVT_PAYLOAD_LEN) {
                handle_nav_pvt(pr->payload);
            }
            /* UBX-ACK-ACK / NAK from CFG commands could be checked here for
             * a more paranoid boot; the small inter-command vTaskDelay()s
             * are sufficient for the BZ251 in practice. */
        }
        break;
    }
}

/* ---------------------------------------------------------------------------
 * FreeRTOS task entry point.
 *
 * Wire it up from main with something like:
 *
 *      xTaskCreate(GPS_Task, "GPS", 1024, NULL,
 *                  tskIDLE_PRIORITY + 2, NULL);
 *
 * Stack: 1 kB is comfortable — peak usage is the 92 B parser payload buffer
 * plus a couple of small locals. 512 B works if you're tight.
 * ------------------------------------------------------------------------- */
void GPS_Task(void *pvParameters)
{
    (void)pvParameters;

    gps_data_mutex = xSemaphoreCreateMutex();
    s_dma_rx_sem   = xSemaphoreCreateBinary();

    memset(&s_parser,         0, sizeof(s_parser));
    memset(&global_gps_data,  0, sizeof(global_gps_data));

    s_gps_uart.uart_base       = LPUART1;
    s_gps_uart.dma_base        = DMA0;
    s_gps_uart.dma_rx_channel  = 0;
    s_gps_uart.dma_tx_channel  = 3;                            /* unused but distinct */
    s_gps_uart.edma_rx_channel = kDma0RequestMuxLpFlexcomm1Rx;
    s_gps_uart.edma_tx_channel = kDma0RequestMuxLpFlexcomm1Tx; /* unused but distinct */
    s_gps_uart.baudrate        = 115200;
    s_gps_uart.enable_dma      = true;
    s_gps_uart.callback        = gps_dma_callback;

    uart_init(&s_gps_uart);

    NVIC_SetPriority(LP_FLEXCOMM1_IRQn, 3);
    NVIC_SetPriority(EDMA_0_CH0_IRQn,   3);

    /* Push the module into 10 Hz binary mode before we start parsing. */
    ubx_configure_gps();

    for (;;) {
        /* EDMA blocks the calling task until the full chunk has been
         * received. At 115200 baud + 10 Hz NAV-PVT (~100 B/100 ms), the
         * 128-byte chunk fills in ~11 ms, so we wake up at ~10 Hz. */
        uart_read_dma(&s_gps_uart, s_dma_rx_buffer, DMA_CHUNK_SIZE);

        if (xSemaphoreTake(s_dma_rx_sem, portMAX_DELAY) == pdTRUE) {
            for (uint32_t i = 0; i < DMA_CHUNK_SIZE; ++i) {
                ubx_parser_feed(&s_parser, s_dma_rx_buffer[i]);
            }
        }
    }
}

/* ---------------------------------------------------------------------------
 * Optional debug telemetry task — prints the latest fix once per second.
 * Keep low priority. Disable by simply not creating the task.
 * ------------------------------------------------------------------------- */
void Tarea_Telemetry(void *pvParameters)
{
    (void)pvParameters;
    gps_data_t local;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (gps_data_mutex == NULL) {
            continue;
        }

        if (xSemaphoreTake(gps_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            local = global_gps_data;
            xSemaphoreGive(gps_data_mutex);

            if (local.is_valid) {
                PRINTF("GPS %dD | SVs:%2d | Lat:%.7f Lon:%.7f | "
                       "Alt(MSL):%7.2fm | GS:%6.2fkt | Hdg:%6.1f deg\r\n",
                       local.fix_type, local.satellites,
                       local.latitude_deg, local.longitude_deg,
                       local.altitude_msl_m,
                       local.ground_speed_knots,
                       local.heading_deg);
            } else {
                PRINTF("GPS searching... SVs:%d\r\n", local.satellites);
            }
        }
    }
}
