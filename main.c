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
#include "vl53l0x.h"
#include "bno_08x.h"


/* SensorTask wakes only on the IMU HINT falling edge in this build —
 * the BMP581 / Kalman filter wiring has been removed.                   */
#define IMU_NOTIFY_BIT     (1u << 0)


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Task priorities. */
#define sensor_task_PRIORITY (configMAX_PRIORITIES - 1)


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void SensorTask(void *pvParameters);


/*******************************************************************************
 * Variables
 ******************************************************************************/
imu_ctrl_t     imu;
vl53l0x_ctrl_t tof;
TaskHandle_t   sensorTaskHandle = NULL;

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
        xTaskNotifyFromISR(sensorTaskHandle, IMU_NOTIFY_BIT,
                           eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*!
 * @brief Timer callback (kept as a stub — timer is initialised in main()
 *        but currently has no consumer in this stripped-down build).
 */
void timer_0_callback(void)
{
}

/*!
 * @brief SH2 sensor callback — decoded inside the bno_08x driver service
 *        loop.  We decode the report and print rotation-vector quaternions
 *        and calibrated angular-velocity samples as they arrive.
 *
 *        Note: the BNO085 does NOT expose angular *acceleration* directly.
 *        SH2_GYROSCOPE_CALIBRATED gives angular *velocity* in rad/s.  If you
 *        want acceleration, differentiate the gyro stream offline.
 */
void sh2_sensor_callback(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t sensorValue;

    if (sh2_decodeSensorEvent(&sensorValue, event) != SH2_OK) {
        return;
    }

    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        // sh2_RotationVectorWAcc_t *rv = &sensorValue.un.rotationVector;
        // /* Tag "R" = rotation: real, i, j, k, accuracy [rad].             */
        // PRINTF("R: %.4f, %.4f, %.4f, %.4f, %.4f\r\n",
        //        rv->real, rv->i, rv->j, rv->k, rv->accuracy);
    }
    else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        sh2_Gyroscope_t *g = &sensorValue.un.gyroscope;
        /* Tag "G" = gyro: x, y, z [rad/s].                               */
        PRINTF("G: %.4f, %.4f, %.4f\r\n", g->x, g->y, g->z);
    }
    else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        // sh2_Accelerometer_t *a = &sensorValue.un.linearAcceleration;
        // /* Tag "A" = linear acceleration (gravity-removed): x, y, z [m/s^2]. */
        // PRINTF("A: %.4f, %.4f, %.4f\r\n", a->x, a->y, a->z);
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
 * @brief Task responsible for processing the IMU reports.  Wakes on the
 *        IMU HINT falling edge, drains all pending packets from the BNO085,
 *        and lets sh2_sensor_callback() print the decoded rotation vector
 *        and gyroscope samples.
 */
static void SensorTask(void *pvParameters)
{
    (void)pvParameters;

    static bool sensors_configured = false;
    uint32_t    notif_value        = 0;
    status_t    result;

    result = bno_08x_start(sh2_sensor_callback);
    if (result != kStatus_Success) {
        PRINTF("BNO085 SH2 session failed to open\r\n");
        vTaskSuspend(NULL);
    }


    for (;;)
    {
        notif_value = 0;
        xTaskNotifyWait(/* ulBitsToClearOnEntry */ 0,
                        /* ulBitsToClearOnExit  */ IMU_NOTIFY_BIT,
                        &notif_value,
                        pdMS_TO_TICKS(100));

        if (notif_value & IMU_NOTIFY_BIT)
        {
            /* Drain all packets the BNO has queued.  Bound the loop so a
             * stuck-LOW HINT can't hang SensorTask.                       */
            int drain_budget = 16;
            do {
                sh2_service();
            } while (gpio_read_input(&imu.gpio_event) == 0 && --drain_budget > 0);
        }

        /* First IMU reset is the trigger to enable the sensor reports. */
        if (!sensors_configured && bno_08x_reset_occurred()) {
            bno_08x_configure_sensors();
            vTaskDelay(pdMS_TO_TICKS(200));   /* let reports start flowing */
            sensors_configured = true;
        }

    }
}
