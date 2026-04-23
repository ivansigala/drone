/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"


/* User includes */
#include "timer_driver_mcxn947.h"
#include "bno_08x.h"
#include "bme280.h"
#include "euler.h"
#include "kalman_z.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SIGN_STR(x) ((x) < 0.0f ? "-" : "")

/* Task priorities. */
#define sensor_task_PRIORITY (configMAX_PRIORITIES - 1)

/* ---------------------------------------------------------------------------
 * Kalman tuning — paste values from your characterisation scripts:
 *   accel_covariance.m    → σ_az² and the accel 2×2 sub-block of Q
 *   accel_bias_allan.m    → q_bias  (per-step bias random-walk variance)
 *   pressure_covariance.m → r_baro
 *
 *   KZ_DT   must match the BNO085 linear-acceleration report interval
 *           (bno_08x_configure_sensors uses 10000 µs → 100 Hz → 0.01 s).
 *
 *   KZ_Q    is the full 3×3 process-noise matrix with an added bias state:
 *
 *             Q = Q_accel + Q_bias
 *
 *             Q_accel = σ_az² · [[dt⁴/4, dt³/2, 0],
 *                                 [dt³/2, dt²,   0],
 *                                 [0,     0,     0]]
 *
 *             Q_bias  =          [[0, 0, 0  ],
 *                                 [0, 0, 0  ],
 *                                 [0, 0, q_b]]
 *
 *   R_BARO  is the measurement noise in altitude variance [m²].
 *
 * The numbers below use σ_az² = 1e-3 and q_bias = 1e-8 as placeholders —
 * replace with measured values from the MATLAB scripts.
 * ------------------------------------------------------------------------- */
#define KZ_DT       0.01f          /* 100 Hz BNO085 → 10 ms predict step     */
#define R_BARO      0.1673287f

static const float32_t KZ_Q[9] = {
    /* row 0:  h / {h, v, b} */
    0.0000e+00f,   1.9000e-10f,   0.0f,
    /* row 1:  v / {h, v, b} */
    1.9000e-10f,   3.8770e-08f,   0.0f,
    /* row 2:  b / {h, v, b}  (this script) */
    0.0f,          0.0f,          6.8263e-09f
};

// static const float32_t KZ_Q[9] = {
//     /* row 0:  h / {h, v, b}  */
//     1.0792e-12f,   2.1584e-10f,   0.0f,
//     /* row 1:  v / {h, v, b}  */
//     2.1584e-10f,   4.3168e-08f,   0.0f,
//     /* row 2:  b / {h, v, b}   ← replace 1.0e-08 with q_bias from Allan script */
//     0.0f,          0.0f,          6.8263e-09f
// };

/* ISA sea-level pressure — used as an initial reference value.  The task
 * overrides this with the very first BME280 sample so altitude comes out
 * as "metres above takeoff" rather than "metres above ISA sea level". */
#define P0_DEFAULT_PA   81325.0f

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void SensorTask(void *pvParameters);

/*******************************************************************************
 * Variables
 ******************************************************************************/
imu_ctrl_t     imu;
bme280_ctrl_t  bme_ctrl;
TaskHandle_t   sensorTaskHandle = NULL;

/* Guarded flag: the SH2 sensor callback must not touch the filter before
 * SensorTask has called kalman_z_init().  The BNO085 only starts emitting
 * reports after bno_08x_configure_sensors(), which happens well after init,
 * but this flag defends against any stray callback during early boot. */
static volatile bool g_kalman_ready = false;

gpio_ctrl_t gpio_time_tracker = {
    .gpio_base = GPIO4,
    .port_base = PORT4,
    .dir = gpio_output,
    .pin = 13
};

/*******************************************************************************
 * Code
 ******************************************************************************/

void IMU_Update_Callback(void)
{
    gpio_clear_interrupt_flag(imu.gpio_event.gpio_base, imu.gpio_event.pin);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (sensorTaskHandle != NULL)
    {
        /* Unblock SensorTask on HINT falling edge. */
        vTaskNotifyGiveFromISR(sensorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*!
 * @brief Timer callback
 */
void timer_0_callback(void)
{

}

void sh2_sensor_callback(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t sensorValue;

    if (sh2_decodeSensorEvent(&sensorValue, event) != SH2_OK) {
        return;
    }

    /* Never call into the filter before it has been initialised. */
    if (!g_kalman_ready) {
        return;
    }

    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        sh2_RotationVectorWAcc_t *rv = &sensorValue.un.rotationVector;
        kalman_z_set_attitude(rv->i, rv->j, rv->k, rv->real);
    }
    else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        sh2_Accelerometer_t *a = &sensorValue.un.linearAcceleration;
        /* Predict step at the 100 Hz linear-accel report rate. */
        kalman_z_predict(a->x, a->y, a->z);
        // PRINTF(": %s%.10f, %s%.10f, %s%.10f\r\n",
        //        SIGN_STR(a->x), a->x,
        //        SIGN_STR(a->y), a->y,
        //        SIGN_STR(a->z), a->z);
        gpio_toggle_output(&gpio_time_tracker);
    }
}

/*!
 * @brief Application entry point.
 */
int main(void)
{
    timer_ctrl_t timer_0 = {
        .timer_id  = 0,
        .frequency = 800
    };

    /* Init board hardware. */
    BOARD_InitHardware();

    gpio_init(&gpio_time_tracker);
    gpio_set_output(&gpio_time_tracker, 0);

    if (xTaskCreate(SensorTask, "Sensor_task", 2048, NULL,
                    sensor_task_PRIORITY, &sensorTaskHandle) != pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }

    PRINTF("Initializing BNO085 IMU...\r\n");
    bno_08x_init(&imu, IMU_Update_Callback);

    timer_init(&timer_0);
    timer_attach_callback(&timer_0, timer_0_callback);

    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for processing the IMU and barometer data,
 *        and running the vertical Kalman filter.
 */
static void SensorTask(void *pvParameters)
{
    (void)pvParameters;

    static bool sensors_configured = false;
    float32_t pa_f32 = 0.0f;
    status_t    result;

    if (bme280_init(&bme_ctrl) != kStatus_Success) {
        PRINTF("BME280 SPI init failed\r\n");
        vTaskSuspend(NULL);
    }
    if (bme280_read_calibration(&bme_ctrl) != kStatus_Success) {
        PRINTF("BME280 calibration read failed\r\n");
        vTaskSuspend(NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(100));   /* let the BME settle before using its readings */
    bme280_parse_data(&bme_ctrl);
    vTaskDelay(pdMS_TO_TICKS(1)); 

    /* -------------------------------------------------------------------
     * Initialise the Kalman filter BEFORE opening the SH2 session so the
     * first sensor callbacks (which can arrive almost immediately once
     * bno_08x_configure_sensors() runs) find a valid filter state.
     * ----------------------------------------------------------------- */
    kalman_z_init(KZ_Q, R_BARO, KZ_DT, (bme_ctrl.data.pressure / 256.0f));
    g_kalman_ready = true;

    result = bno_08x_start(sh2_sensor_callback);
    if (result != kStatus_Success) {
        PRINTF("BNO085 SH2 session failed to open\r\n");
        vTaskSuspend(NULL);
    }

    for (;;)
    {
        /* Block until HINT falling edge fires the notification.
         * The 100 ms timeout also lets us poll for reset events. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        /* Drain all packets the BNO has queued.  Bound the loop so a
         * stuck-LOW HINT can't hang SensorTask. */
        int drain_budget = 16;
        do {
            sh2_service();
        } while (gpio_read_input(&imu.gpio_event) == 0 && --drain_budget > 0);

        /* Pull fresh compensated pressure + temperature from the BME280. */
        bme280_parse_data(&bme_ctrl);

        /* Reject obviously-bad baro samples (e.g. the zero-filled first
         * reading before the BME has valid data, or sensor glitches).   */
        pa_f32 = (float32_t)bme_ctrl.data.pressure / 256.0f;
        if (pa_f32 > 50000.0f && pa_f32 < 110000.0f) {    /* sanity */
            /* Kalman update from the new baro sample. */
            kalman_z_update(bme_ctrl.data.pressure,
                            bme_ctrl.data.temperature);
        }

        /* First IMU reset is the trigger to enable the sensor reports. */
        if (!sensors_configured && bno_08x_reset_occurred()) {
            bno_08x_configure_sensors();
            vTaskDelay(pdMS_TO_TICKS(200));   /* let reports start flowing */
            sensors_configured = true;
        }

        /* ---------------------------------------------------------------
         * Print relative altitude [m], vertical velocity [m/s] and the
         * estimated accel bias [m/s²].
         *
         *   altitude > 0  →  drone is above takeoff height
         *   velocity > 0  →  drone is ascending
         *   bias     →      persistent world-Z accel offset the filter is
         *                   absorbing (should converge to a small constant
         *                   a few seconds after bootup when stationary).
         *
         * Format matches the "<tag>: v1, v2, v3" pattern used by the
         * MATLAB covariance scripts, so the same parser also works for
         * Kalman telemetry capture.
         * ------------------------------------------------------------- */
        float32_t alt_m  = kalman_z_get_altitude();
        float32_t vel_m  = kalman_z_get_velocity();
        float32_t bias_m = kalman_z_get_accel_bias();
        PRINTF(": %s%.4f, %s%.4f, %s%.4f\r\n",
               SIGN_STR(alt_m),  alt_m,
               SIGN_STR(vel_m),  vel_m,
               SIGN_STR(bias_m), bias_m);
    }
}