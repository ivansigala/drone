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
#include "bno_08x.h"
#include "euler.h"

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
 * Prototypes
 ******************************************************************************/
imu_ctrl_t imu;
TaskHandle_t sensorTaskHandle = NULL;

/*******************************************************************************
 * Code
 ******************************************************************************/

void IMU_Update_Callback(void)
{   
    gpio_clear_interrupt_flag(imu.gpio_event.gpio_base, imu.gpio_event.pin);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (sensorTaskHandle != NULL)
    {
        // Unblock the SensorTask to let it know the DMA transfer is done
        vTaskNotifyGiveFromISR(sensorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void sh2_sensor_callback(void *cookie, sh2_SensorEvent_t *event) {
    sh2_SensorValue_t sensorValue;

    if (sh2_decodeSensorEvent(&sensorValue, event) == SH2_OK) {

        if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {

            sh2_RotationVectorWAcc_t *rv = &sensorValue.un.rotationVector;

            float roll, pitch, yaw;
            q_to_ypr(rv->real, rv->i, rv->j, rv->k, &yaw, &pitch, &roll);

            // Convert radians to degrees
            float rad2deg = 180.0f / 3.14159265f;

            roll = roll * rad2deg;
            pitch = pitch * rad2deg;
            yaw = yaw * rad2deg;
            PRINTF("Yaw=%.2f, Pitch=%.2f, Roll=%.2f\r\n", yaw, pitch, roll);

        }

        if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {

            //sh2_Accelerometer_t *accel_data = &sensorValue.un.linearAcceleration;

        
        }
    }
}

/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitHardware();

    if (xTaskCreate(SensorTask , "Sensor_task", 1024, NULL, sensor_task_PRIORITY, &sensorTaskHandle) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }

    PRINTF("Initializing BNO085 IMU...\r\n");

    bno_08x_init(&imu, NULL, IMU_Update_Callback);

    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for processing the IMU data.
 *
 * After each HINT interrupt, we call sh2_service() in a loop until HINT goes
 * high.  The CEVA library may need multiple read/write cycles per interrupt
 * (e.g. reading an advertisement, then sending a command in response).
 * Sensor configuration is deferred until the BNO085 signals SH2_RESET.
 */
static void SensorTask(void *pvParameters)
{
    static bool sensors_configured = false;
    status_t result;

    // Open the SH2 session here, inside a task context, so that
    // hal_getTimeUs() works (it needs the FreeRTOS scheduler running).
    // sh2_open() has a blocking poll loop with a 200ms timeout that
    // reads the BNO085 boot packets and waits for reset-complete.
    result = bno_08x_start(sh2_sensor_callback);
    if (result != kStatus_Success) {
        PRINTF("BNO085 SH2 session failed to open\r\n");
        vTaskSuspend(NULL);
    }

    PRINTF("bno_08x_start returned %d\r\n", result);  // should be 0

    // In bno_08x_configure_sensors, before sh2_setSensorConfig:
    PRINTF("Calling sh2_setSensorConfig, resetComplete state unknown\r\n");

    for (;;)
    {
        // Block until HINT falling edge fires the notification.
        // Use a timeout so we can also poll for reset events periodically.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        // Service the SH2 library repeatedly while HINT is asserted.
        // Each sh2_service() call processes one SHTP packet.
        // Loop until HINT goes high (no more data from BNO085).
        do {
            sh2_service();
        } while (gpio_read_input(&imu.gpio_event) == 0);

        // Once the BNO085 signals reset-complete, configure sensors
        if (!sensors_configured && bno_08x_reset_occurred()) {
            PRINTF("BNO085 reset complete, configuring sensors...\r\n");
            bno_08x_configure_sensors();
            vTaskDelay(pdMS_TO_TICKS(200)); // let a few reports arrive first
            bno_08x_tare();
            sensors_configured = true;
        }
    }
}