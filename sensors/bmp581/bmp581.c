/*
    bmp581.c
    Author: Diego
    Created on: 30, April 2026

    Driver for the Bosch BMP581 barometric pressure / temperature sensor,
    talking to the same LPSPI bus the BME280 driver used (see
    spi_get_defaultconfig_bar() in spi_driver_mcxn947.c).

    The BMP581 ships compensation on-chip, so the BME280 calibration-trim
    read is replaced by a much smaller config block (OSR/ODR/IIR/power).
    The data registers already hold engineering values:

         temperature : 24-bit signed,  Q8.16 °C    (raw / 2^16 → °C)
         pressure    : 24-bit unsigned, Q18.6 Pa   (raw / 2^6  → Pa)

    To stay compatible with the existing kalman_z pipeline, this driver
    re-scales the values into the same fixed-point format the BME280
    driver outputs:

         data.temperature : int32_t, 0.01 °C units
         data.pressure    : int64_t, Q24.8 Pa units (= BME280 format ×1)

    SPI framing (verified empirically against a logic-analyzer capture
    of a CHIP_ID read): with this LPSPI driver / SPI mode combo the
    BMP581 does NOT insert a dummy byte between the address phase and
    the first data byte.  The MISO byte clocked in during the address
    phase is junk (rx[0]), and the very next byte (rx[1]) is the first
    real data byte.  This matches how the BME280 driver on the same bus
    behaves and contradicts the datasheet's §7.1.2 dummy-byte note for
    this hardware setup.
 */

#include "bmp581.h"
#include "bmp581_reg.h"
#include "fsl_debug_console.h"
#include "task.h"


/*************************************
 Global Variables
*************************************/
bmp581_ctrl_t *g_bmp_barometer;


AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_bmp_tx[16], 4);
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_bmp_rx[16], 4);


/*************************************
 Private helpers
*************************************/

/*!
 * @brief Writes a single byte to a BMP581 register over SPI.
 *        Frame: { reg & 0x7F, val }   (identical to BME280 write).
 */
static status_t bmp581_write_reg(bmp581_ctrl_t *bmp, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & BMP581_SPI_WR_MASK, val };
    return spi_master_transfer(&bmp->spi_ctrl, tx, NULL, sizeof(tx));
}

/*!
 * @brief Reads a single byte from a BMP581 register over SPI.
 *
 * Two-byte transfer (no dummy byte on this bus, see file header note):
 *
 *     tx[0] = reg | 0x80    rx[0] = junk (MISO during address phase)
 *     tx[1] = 0x00          rx[1] = register data
 */
static status_t bmp581_read_reg(bmp581_ctrl_t *bmp, uint8_t reg, uint8_t *out)
{
    s_bmp_tx[0] = reg | BMP581_SPI_RD_MASK;
    s_bmp_tx[1] = 0x00;
    s_bmp_rx[0] = 0;
    s_bmp_rx[1] = 0;

    status_t status = spi_master_transfer(&bmp->spi_ctrl, s_bmp_tx, s_bmp_rx, 2U);
    if (status == kStatus_Success) {
        *out = s_bmp_rx[1];
    }
    return status;
}


status_t bmp581_init(bmp581_ctrl_t *bmp_ctl, void *gpio_callback)
{
    status_t status;
    uint8_t  chip_id    = 0;
    uint8_t  int_status = 0;

#ifdef MCXN947
    /* Re-use the same default barometer SPI configuration the BME280
     * driver uses (LPSPI2, 1 MHz, eDMA channels 9/10).  No pin or clock
     * changes — same physical bus, same chip-select.                       */
    spi_get_defaultconfig_bar(&bmp_ctl->spi_ctrl);
#endif

    g_bmp_barometer = bmp_ctl;

    NVIC_SetPriority(EDMA_0_CH9_IRQn, 5);
    NVIC_SetPriority(EDMA_0_CH10_IRQn, 5);

    status = spi_init(&(bmp_ctl->spi_ctrl));
    if (status != kStatus_Success)
    {
        return status;
    }

    /* --- Verify SPI communication by reading the chip ID ---
     * BMP581 always returns 0x50 at register 0x01.                          */
    status = bmp581_read_reg(bmp_ctl, BMP581_CHIP_ID_REG, &chip_id);
    if (status != kStatus_Success)
    {
        return status;
    }
    if (chip_id != BMP581_CHIP_ID)
    {
        PRINTF("BMP581: unexpected chip ID 0x%02X (expected 0x50)\r\n", chip_id);
        return kStatus_Fail;
    }

    /* --- Soft reset — datasheet §5.7 ---
     * Writing 0xB6 to CMD (0x7E) restores POR defaults.  After issuing the
     * reset we MUST wait until STATUS.NVM_RDY = 1 before doing anything
     * else: until the on-chip trim coefficients have been re-loaded from
     * NVM the compensation pipeline is fed garbage and the data registers
     * return uncalibrated readings (very stable but completely wrong).    */
    status = bmp581_write_reg(bmp_ctl, BMP581_CMD_REG, BMP581_SOFT_RESET_CMD);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Brief settle delay so the chip latches the reset before we poll.    */
    vTaskDelay(pdMS_TO_TICKS(2));

    /* Poll STATUS.NVM_RDY with a generous timeout.  Datasheet quotes
     * t_nvm ≤ 2 ms typical; we allow ~50 ticks (~50 ms at the typical
     * FreeRTOS configTICK_RATE_HZ = 1000) to absorb any worst-case
     * NVM ECC retries.                                                   */
    {
        uint8_t  status_reg = 0;
        uint32_t tries      = 50U;
        while (tries-- > 0U)
        {
            status = bmp581_read_reg(bmp_ctl, BMP581_STATUS_REG, &status_reg);
            if (status != kStatus_Success)
            {
                return status;
            }
            if ((status_reg & BMP581_STATUS_NVM_RDY) != 0U)
            {
                /* NVM trim loaded — verify no error before we trust it. */
                if ((status_reg & (BMP581_STATUS_NVM_ERR | BMP581_STATUS_NVM_CMD_ERR)) != 0U)
                {
                    PRINTF("BMP581: NVM error after reset (STATUS=0x%02X)\r\n",
                           status_reg);
                    return kStatus_Fail;
                }
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if ((status_reg & BMP581_STATUS_NVM_RDY) == 0U)
        {
            PRINTF("BMP581: NVM_RDY timeout (STATUS=0x%02X) — compensation "
                   "trim never loaded; data registers will be garbage.\r\n",
                   status_reg);
            return kStatus_Fail;
        }
    }

    /* INT_STATUS.por bit must be 1 after a successful POR/soft-reset.
     * Reading INT_STATUS clears it, so this also wipes any stale flags
     * before we move into NORMAL mode.                                     */
    status = bmp581_read_reg(bmp_ctl, BMP581_INT_STATUS_REG, &int_status);
    if (status != kStatus_Success)
    {
        return status;
    }
    if ((int_status & BMP581_INT_STATUS_POR) == 0U)
    {
        PRINTF("BMP581: POR flag not set after soft reset (INT_STATUS=0x%02X)\r\n",
               int_status);
        return kStatus_Fail;
    }

    /* --- Configure the DSP / IIR / OSR chain ---------------------------
     *
     * Order matters: the device must be in STANDBY for OSR/IIR writes.
     * After a soft reset PWR_MODE is already 00 (standby), so we can
     * write the configuration block first and then flip PWR_MODE to
     * NORMAL with the final ODR_CONFIG write.
     *
     *   DSP_CONFIG : route shadow pressure register through the IIR
     *   DSP_IIR    : pressure IIR coef = 127, temperature IIR bypassed
     *   OSR_CONFIG : OSR_P = ×8, OSR_T = ×1, PRESS_EN = 1
     *   INT_SOURCE : DRDY data-ready is the only enabled interrupt source
     *   INT_CONFIG : active-low, push-pull, pulsed, INT pin enabled
     *   ODR_CONFIG : ODR ≈ 50 Hz, DEEP_DIS = 1, PWR_MODE = NORMAL
     * ------------------------------------------------------------------- */
    status = bmp581_write_reg(bmp_ctl, BMP581_DSP_CONFIG_REG, BMP581_DSP_CONFIG_VAL);
    if (status != kStatus_Success)
    {
        return status;
    }
    status = bmp581_write_reg(bmp_ctl, BMP581_DSP_IIR_REG, BMP581_DSP_IIR_VAL);
    if (status != kStatus_Success)
    {
        return status;
    }
    status = bmp581_write_reg(bmp_ctl, BMP581_OSR_CONFIG_REG, BMP581_OSR_CONFIG_VAL);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* INT_SOURCE must be written BEFORE INT_CONFIG so that when the INT
     * pin is enabled the only thing it can fire on is DRDY.              */
    status = bmp581_write_reg(bmp_ctl, BMP581_INT_SOURCE_REG, BMP581_INT_SOURCE_VAL);
    if (status != kStatus_Success)
    {
        return status;
    }
    status = bmp581_write_reg(bmp_ctl, BMP581_INT_CONFIG_REG, BMP581_INT_CONFIG_VAL);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Writing ODR_CONFIG with PWR_MODE=NORMAL kicks off continuous output. */
    status = bmp581_write_reg(bmp_ctl, BMP581_ODR_CONFIG_REG, BMP581_ODR_CONFIG_VAL);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* --- Wire up the data-ready GPIO interrupt -------------------------
     *
     * The caller has already populated bmp_ctl->gpio_event with the
     * gpio_base / port_base / pin describing the board net connected to
     * the BMP581 INT pin.  We initialise it as an input and attach the
     * caller-supplied falling-edge callback.  The first DRDY pulse may
     * happen as soon as ~one ODR period from now (≈20 ms at 50 Hz).
     *
     * If the caller passes NULL for gpio_callback we skip GPIO setup
     * entirely — useful for unit tests or polling-only experiments,
     * but not the recommended path for the flight loop.
     * ------------------------------------------------------------------- */
    if (gpio_callback != NULL)
    {
        bmp_ctl->gpio_event.dir = gpio_input;
        gpio_init(&bmp_ctl->gpio_event);
        gpio_attach_interrupt(&bmp_ctl->gpio_event, gpio_callback);
    }

    /* --- Config read-back diagnostic ------------------------------------
     * Verifies the configuration block actually committed to silicon.
     * Expected: INT=0x08  SRC=0x01  OSR=0x58  ODR=0xB5
     * STATUS expected with NVM_RDY=1 and CORE_RDY=1 (low nibble = 0x03).
     * INT_STATUS bit 0 (DRDY) flickers at the ODR — if you see it = 0
     * here that's fine (we just read INT_STATUS to clear POR earlier).
     * Remove or #if-out once the bring-up is confirmed.                    */
    {
        uint8_t v_int = 0, v_src = 0, v_osr = 0, v_odr = 0;
        uint8_t v_dsp = 0, v_iir = 0, v_sta = 0, v_ist = 0;
        bmp581_read_reg(bmp_ctl, BMP581_INT_CONFIG_REG, &v_int);
        bmp581_read_reg(bmp_ctl, BMP581_INT_SOURCE_REG, &v_src);
        bmp581_read_reg(bmp_ctl, BMP581_OSR_CONFIG_REG, &v_osr);
        bmp581_read_reg(bmp_ctl, BMP581_ODR_CONFIG_REG, &v_odr);
        bmp581_read_reg(bmp_ctl, BMP581_DSP_CONFIG_REG, &v_dsp);
        bmp581_read_reg(bmp_ctl, BMP581_DSP_IIR_REG,    &v_iir);
        bmp581_read_reg(bmp_ctl, BMP581_STATUS_REG,     &v_sta);
        bmp581_read_reg(bmp_ctl, BMP581_INT_STATUS_REG, &v_ist);
        PRINTF("BMP581 cfg readback:\r\n"
               "  INT=0x%02X SRC=0x%02X OSR=0x%02X ODR=0x%02X "
               "DSP=0x%02X IIR=0x%02X\r\n"
               "  expect    0x08      0x01      0x58      0xB5      "
               "0x08      0x38\r\n"
               "  STATUS=0x%02X (NVM_RDY=%u CORE_RDY=%u NVM_ERR=%u)  "
               "INT_STATUS=0x%02X\r\n",
               v_int, v_src, v_osr, v_odr, v_dsp, v_iir,
               v_sta,
               (unsigned)((v_sta & BMP581_STATUS_NVM_RDY)  ? 1U : 0U),
               (unsigned)((v_sta & BMP581_STATUS_CORE_RDY) ? 1U : 0U),
               (unsigned)((v_sta & BMP581_STATUS_NVM_ERR)  ? 1U : 0U),
               v_ist);
    }

    //PRINTF("BMP581: NORMAL mode @50Hz (osr P×8, T×1, IIR P=127, INT=DRDY)\r\n");
    return kStatus_Success;
}


void bmp581_parse_data(bmp581_ctrl_t *bmp_ctl)
{
    /* -----------------------------------------------------------
     * Burst-read 6 output bytes starting at 0x1D.  With NO dummy
     * byte (see file header note) the layout is:
     *
     *   tx[0] = 0x1D | 0x80       rx[0] = junk (addr-phase MISO)
     *   tx[1] = 0x00              rx[1] = temp_xlsb     0x1D
     *   tx[2] = 0x00              rx[2] = temp_lsb      0x1E
     *   tx[3] = 0x00              rx[3] = temp_msb      0x1F
     *   tx[4] = 0x00              rx[4] = press_xlsb    0x20
     *   tx[5] = 0x00              rx[5] = press_lsb     0x21
     *   tx[6] = 0x00              rx[6] = press_msb     0x22
     *
     * (Total transfer = 1 addr + 6 data = 7 bytes.)
     * ----------------------------------------------------------- */
    const size_t transfer_len = 1U + BMP581_DATA_BURST_LEN;   /* = 7 */

    uint8_t tx[1U + BMP581_DATA_BURST_LEN] = {0};
    uint8_t rx[1U + BMP581_DATA_BURST_LEN] = {0};

    tx[0] = BMP581_DATA_BURST_START | BMP581_SPI_RD_MASK;

    if (spi_master_transfer(&bmp_ctl->spi_ctrl, tx, rx, transfer_len) != kStatus_Success)
    {
        return;
    }

    /* Reconstruct the 24-bit raw values.  Layout is little-endian:
     * the lowest-address byte (XLSB) is the LSB of the 24-bit word.       */
    uint32_t raw_temp_u24  = ((uint32_t)rx[3] << 16) |
                             ((uint32_t)rx[2] <<  8) |
                              (uint32_t)rx[1];
    uint32_t raw_press_u24 = ((uint32_t)rx[6] << 16) |
                             ((uint32_t)rx[5] <<  8) |
                              (uint32_t)rx[4];

    /* Temperature is signed 24-bit — sign-extend to int32_t.
     * (Left-shift then arithmetic right-shift is the canonical trick.)    */
    int32_t adc_temp = (int32_t)(raw_temp_u24 << 8) >> 8;

    /* -------- Convert to the same fixed-point units the BME280 uses ---
     *
     *   Temperature:
     *     raw_temp / 2^16 = °C
     *     × 100 to get 0.01 °C units → match BME280 driver
     *     →  data.temperature = (raw_temp * 100) >> 16
     *
     *     Use a 64-bit intermediate to avoid overflow at the upper end of
     *     the operating range (raw can be ~±10^7, ×100 → ~±10^9 — still
     *     fits in int32 but the intermediate (raw_temp*100) before the
     *     shift can overshoot the int32 limit when raw_temp is near its
     *     maximum positive value, so play it safe).
     *
     *   Pressure:
     *     raw_press / 2^6 = Pa     (BMP581 native Q18.6)
     *     The BME280 driver reports pressure in Q24.8 Pa.  Q18.6 → Q24.8
     *     means multiplying by 4 (= shifting left by 2):
     *     →  data.pressure = (int64_t)raw_press << 2
     * ------------------------------------------------------------------ */
    bmp_ctl->data.temperature = (int32_t)(((int64_t)adc_temp * 100) >> 16);
    bmp_ctl->data.pressure    = ((int64_t)raw_press_u24) << 2;
}
