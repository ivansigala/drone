/*
 * imu_kalman_z.c
 *
 * Self-contained BNO085 (IMU) + VLX (ESP32 SPI-slave VL53L0X bridge) +
 * Kalman-Z (vertical-axis state estimator) integration for the FreeRTOS
 * drone firmware.
 *
 * The entire pipeline lives in this single translation unit. main.c is
 * NOT modified; instead this file uses a `__attribute__((constructor))`
 * hook that the ARM GCC C startup (__libc_init_array, called from
 * Reset_Handler before main) invokes automatically. That hook only
 * creates the SensorTask via xTaskCreate -- it does not touch hardware.
 *
 * Updated: 18, May 2026 — measurement source switched from BME280
 *                          hypsometric altitude to a fixed-base VLX
 *                          (VL53L0X / ESP32 bridge) reporting absolute
 *                          world-Z position in millimetres.  Tuning
 *                          constants now match
 *                          frdmmcxn947_freertos_imu/main.c verbatim.
 *
 * Boot order
 * ----------
 *   1. Reset_Handler   -> .bss zeroed, .data copied, SystemInit.
 *   2. __libc_init_array runs `sensor_fusion_register` (constructor).
 *      It calls xTaskCreate() -- safe because heap_4 init is lazy and
 *      task bodies do not execute until vTaskStartScheduler() is called.
 *   3. main() runs BOARD_InitHardware (clocks + pin mux including the
 *      LPSPI2/3 pins we added), debug_uart_init, rc_init, dshot_init,
 *      timer_init, then vTaskStartScheduler().
 *   4. SensorTask wakes up. As its FIRST action it calls bno_08x_init
 *      (now safe because clocks + pins are configured), then
 *      vlx_init + kalman_z_init + bno_08x_start.
 *   5. Steady state: SH2 service on IMU HINT, VLX measurement update,
 *      and a 50 Hz binary frame (DEBUG_UART_FRAME_ID_KALMAN_Z = 0x04)
 *      carrying ts_ms / z_m / v_m_s.
 *
 * Kalman tuning constants (KZ_DT, KZ_R_Z, KZ_Q) match
 * frdmmcxn947_freertos_imu/main.c verbatim. Keep them in sync if you
 * re-tune the filter there.
 *
 * Author: Diego, 2026
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>      /* memcpy for binary frame packing */

#include "FreeRTOS.h"
#include "task.h"

#include "fsl_common.h"
#include "fsl_debug_console.h"   /* PRINTF used by sensor drivers on hard errors */

#include "bno_08x.h"
#include "vlx_esp32.h"
#include "kalman_z.h"
#include "timer_driver_mcxn947.h"
#include "debug_uart_mcxn947.h"
#include "gpio_driver_mcxn947.h"

/* ---------------------------------------------------------------------------
 * Task config
 * ------------------------------------------------------------------------- */
#define SENSOR_TASK_PRIORITY      4
#define SENSOR_TASK_STACK_WORDS   2048

/* ---------------------------------------------------------------------------
 * Sensor task notification bits + Kalman-Z tuning
 *
 *   IMU_NOTIFY_BIT  raised by the BNO085 HINT-falling-edge ISR (drives
 *                   the SH2 service path inside SensorTask).
 *
 *   KZ_DT, KZ_R_Z, KZ_Q match the IMU project exactly. See
 *   frdmmcxn947_freertos_imu/main.c for the derivation
 *   (Allan-variance / VLX-covariance MATLAB scripts).
 * ------------------------------------------------------------------------- */
#define IMU_NOTIFY_BIT            (1u << 0)

#define KZ_DT                     0.010f          /* BNO linear-accel period: 100 Hz */
#define KZ_R_Z                    2.427409e-06f   /* var(z_meas), VLX 1σ ≈ 0.0016 m   */

static const float32_t KZ_Q[9] = {
    /* row 0: h / {h, v, b}    —   σ_az²·dt⁴/4,  σ_az²·dt³/2,  0      */
    1.054996e-13f,   2.109992e-11f,   0.0f,
    /* row 1: v / {h, v, b}    —   σ_az²·dt³/2,  σ_az²·dt²,    0      */
    2.109992e-11f,   4.219983e-09f,   0.0f,
    /* row 2: b / {h, v, b}    —   0,            0,            q_bias */
    0.0f,            0.0f,            1.144956e-08f
};

/* Debug-UART print rate for the Kalman-Z output. 50 Hz = every 20 ms. */
#define KALMAN_Z_PRINT_PERIOD_MS  20U

/* ---------------------------------------------------------------------------
 * Module-private state (no global externs -- nothing else needs to see this)
 * ------------------------------------------------------------------------- */
static imu_ctrl_t    s_imu;
static vlx_ctrl_t    s_vlx;
static TaskHandle_t  s_sensor_task   = NULL;

/* Gate that protects the Kalman filter against stray SH2 callbacks that
 * arrive before SensorTask has finished kalman_z_init(). */
static volatile bool s_kalman_ready  = false;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * ------------------------------------------------------------------------- */
static void SensorTask(void *pvParameters);
static void IMU_Update_Callback(void);
static void sh2_sensor_callback(void *cookie, sh2_SensorEvent_t *event);

/* =========================================================================
 *  ISR + SH2 plumbing
 * ======================================================================= */

/*!
 * @brief BNO085 HINT falling-edge ISR -- forwards to SensorTask via a
 *        task notification so the SH2 service path runs at task priority.
 */
static void IMU_Update_Callback(void)
{
    gpio_clear_interrupt_flag(s_imu.gpio_event.gpio_base, s_imu.gpio_event.pin);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (s_sensor_task != NULL)
    {
        xTaskNotifyFromISR(s_sensor_task, IMU_NOTIFY_BIT,
                           eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*!
 * @brief SH2 sensor callback -- decoded inside the bno_08x driver service
 *        loop.  Drives the Kalman filter:
 *
 *          SH2_ROTATION_VECTOR     -> kalman_z_set_attitude (body->world)
 *          SH2_LINEAR_ACCELERATION -> kalman_z_predict      (100 Hz)
 *
 *        Both branches are gated behind s_kalman_ready so the filter is
 *        only touched after SensorTask has finished kalman_z_init().
 *
 *        The VLX measurement update is driven from SensorTask, not here,
 *        because the VLX is polled outside the SH2 service loop.
 */
static void sh2_sensor_callback(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t sensorValue;

    if (sh2_decodeSensorEvent(&sensorValue, event) != SH2_OK) {
        return;
    }
    if (!s_kalman_ready) {
        return;
    }

    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        sh2_RotationVectorWAcc_t *rv = &sensorValue.un.rotationVector;
        kalman_z_set_attitude(rv->i, rv->j, rv->k, rv->real);
    }
    else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        sh2_Accelerometer_t *a = &sensorValue.un.linearAcceleration;
        kalman_z_predict(a->x, a->y, a->z);
    }
}

/* =========================================================================
 *  SensorTask
 * ======================================================================= */

/*!
 * @brief Sensor task -- runs the BNO085 IMU + VLX (ESP32 / VL53L0X) bridge
 *        through the Kalman-Z filter and streams the estimated vertical
 *        state (altitude, velocity) over the debug UART at 50 Hz.
 *
 * Wake sources
 * ------------
 *   - IMU HINT falling edge (via IMU_Update_Callback) -> SH2 service.
 *     Inside sh2_service(), sh2_sensor_callback() pushes new quaternions
 *     and linear acceleration into the Kalman filter.
 *   - 20 ms wait timeout: clocks one SPI transaction against the ESP32
 *     VLX bridge and drives the 50 Hz print cadence.
 *
 * Filter wiring (mirrors frdmmcxn947_freertos_imu/main.c::SensorTask)
 * -------------------------------------------------------------------
 *   1. bno_08x_init                    -- IMU SPI + reset + HINT IRQ.
 *   2. vlx_init                        -- LPSPI2 master to ESP32 slave.
 *   3. kalman_z_init(Q, R_z, dt)       -- 3-arg signature: reference is
 *                                          auto-latched on the first
 *                                          kalman_z_update() call.
 *   4. bno_08x_start                   -- opens the SH2 session.
 *   5. First SH2 reset event           -- bno_08x_configure_sensors()
 *                                         enables ROTATION_VECTOR +
 *                                         LINEAR_ACCELERATION at 100 Hz.
 *   6. Every wakeup: drain SH2 packets, read VLX, kalman_z_update().
 *   7. Every 20 ms: emit DEBUG_UART_FRAME_ID_KALMAN_Z frame:
 *
 *         bytes 0..3   uint32  ts_ms          LPTMR0 timestamp [ms]
 *         bytes 4..7   float32 z_kalman_m     filtered altitude [m]
 *         bytes 8..11  float32 v_kalman_m_s   filtered vertical velocity
 */
static void SensorTask(void *pvParameters)
{
    (void)pvParameters;

    static bool sensors_configured = false;
    uint32_t    notif_value        = 0;
    uint32_t    last_print_ms      = 0;

    /* --- IMU hardware bring-up (SPI + reset + HINT interrupt) --------- */
    if (bno_08x_init(&s_imu, IMU_Update_Callback) != kStatus_Success) {
        debug_uart_print("BNO085 init failed\r\n");
        vTaskSuspend(NULL);
    }

    /* --- VLX (ESP32 SPI-slave) bring-up ------------------------------- */
    if (vlx_init(&s_vlx) != kStatus_Success) {
        debug_uart_print("VLX SPI init failed\r\n");
        vTaskSuspend(NULL);
    }

    /* Let the ESP32 finish booting and start staging frames before the
     * first vlx_read() in the main loop.                                  */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Initialise the Kalman filter BEFORE opening the SH2 session, so
     * the first BNO callbacks find a valid filter state. s_kalman_ready
     * is the gate that the SH2 callback checks.
     *
     * The reference altitude is auto-latched on the first
     * kalman_z_update() call, so no priming sample is needed here.        */
    kalman_z_init(KZ_Q, KZ_R_Z, KZ_DT);
    s_kalman_ready = true;

    /* --- IMU SH2 session --------------------------------------------- */
    if (bno_08x_start(sh2_sensor_callback) != kStatus_Success) {
        debug_uart_print("BNO085 SH2 session failed to open\r\n");
        vTaskSuspend(NULL);
    }

    last_print_ms = timer_get_ms();

    for (;;)
    {
        notif_value = 0;
        xTaskNotifyWait(/* ulBitsToClearOnEntry */ 0,
                        /* ulBitsToClearOnExit  */ IMU_NOTIFY_BIT,
                        &notif_value,
                        pdMS_TO_TICKS(KALMAN_Z_PRINT_PERIOD_MS));

        if (notif_value & IMU_NOTIFY_BIT)
        {
            /* Drain all packets queued by the BNO. Bound the loop so a
             * stuck-LOW HINT can't hang SensorTask.                      */
            int drain_budget = 16;
            do {
                sh2_service();
            } while (gpio_read_input(&s_imu.gpio_event) == 0 && --drain_budget > 0);
        }

        /* VLX sample -> Kalman measurement update.  vlx_read() validates
         * the magic word internally; on a corrupted frame it returns Fail
         * and leaves s_vlx.data untouched so we simply skip the update.
         * The ESP32 also stamps a per-sample status, which we cross-check
         * before driving the filter.                                      */
        status_t vlx_status = vlx_read(&s_vlx);
        if (vlx_status == kStatus_Success &&
            s_vlx.data.status == VLX_STATUS_OK)
        {
            const float32_t z_raw_m = (float32_t)s_vlx.data.distance_mm * 0.001f;
            kalman_z_update(z_raw_m);
        }

        /* First IMU reset is the trigger to enable the sensor reports. */
        if (!sensors_configured && bno_08x_reset_occurred()) {
            bno_08x_configure_sensors();
            vTaskDelay(pdMS_TO_TICKS(200));   /* let reports start flowing */
            sensors_configured = true;
        }

        /* 50 Hz Kalman-Z output stream over the framed debug UART. */
        uint32_t now_ms = timer_get_ms();
        if ((now_ms - last_print_ms) >= KALMAN_Z_PRINT_PERIOD_MS)
        {
            last_print_ms = now_ms;

            float    z_m   = kalman_z_get_altitude();
            float    v_m_s = kalman_z_get_velocity();
            uint8_t  payload[12];

            memcpy(&payload[0], &now_ms, sizeof(now_ms));   /* 4 */
            memcpy(&payload[4], &z_m,    sizeof(z_m));      /* 4 */
            memcpy(&payload[8], &v_m_s,  sizeof(v_m_s));    /* 4 */

            (void)debug_uart_send_frame(DEBUG_UART_FRAME_ID_KALMAN_Z,
                                        payload, sizeof(payload));
        }
    }
}

/* =========================================================================
 *  Self-bootstrap hook
 *
 *  Runs from __libc_init_array (invoked by Reset_Handler before main).
 *  Creates the SensorTask via xTaskCreate. The task does NOT start
 *  executing here -- vTaskStartScheduler() inside main() is what
 *  eventually lets it run, by which point BOARD_InitHardware has
 *  configured the clocks and pin mux that bno_08x_init / vlx_init
 *  depend on.
 *
 *  Why this works
 *  --------------
 *    - heap_4 uses a static ucHeap[] buffer. .bss has been zeroed by
 *      Reset_Handler, so prvHeapInit can run lazily on the first
 *      pvPortMalloc.
 *    - xTaskCreate only touches heap + the ready-list -- no hardware.
 *    - The SensorTask body defers all hardware-dependent calls to its
 *      own context, which runs after main() has booted the board.
 *
 *  If the task can't be created (heap exhausted), s_sensor_task stays
 *  NULL and IMU_Update_Callback simply ignores the IMU HINT events.
 * ======================================================================= */
__attribute__((used, constructor))
static void sensor_fusion_register(void)
{
    (void)xTaskCreate(SensorTask,
                      "sensor_task",
                      SENSOR_TASK_STACK_WORDS,
                      NULL,
                      SENSOR_TASK_PRIORITY,
                      &s_sensor_task);
}
