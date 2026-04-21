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
#include "bme280.h"
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
bme280_ctrl_t bme_ctrl;
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
            // PRINTF("Yaw=%.2f, Pitch=%.2f, Roll=%.2f\r\n", yaw, pitch, roll);

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

    if (xTaskCreate(SensorTask , "Sensor_task", 2048, NULL, sensor_task_PRIORITY, &sensorTaskHandle) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }

    PRINTF("Initializing BNO085 IMU...\r\n");

    bno_08x_init(&imu, IMU_Update_Callback);

    
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for processing the IMU data.
 *
 * After each HINT interrupt, we call sh2_service() in a loop until HINT goes
 * high. And read the barometer data over SPI
 */
static void SensorTask(void *pvParameters)
{
    static bool sensors_configured = false;
    //float temp_C, press_Pa, hum_pct;
    status_t result;

    if (bme280_init(&bme_ctrl) != kStatus_Success)
    {
        PRINTF("BME280 SPI init failed\r\n");
        vTaskSuspend(NULL);
    }
    if (bme280_read_calibration(&bme_ctrl) != kStatus_Success)
    {
        PRINTF("BME280 calibration read failed\r\n");
        vTaskSuspend(NULL);
    }

    result = bno_08x_start(sh2_sensor_callback);
    if (result != kStatus_Success) {
        PRINTF("BNO085 SH2 session failed to open\r\n");
        vTaskSuspend(NULL);
    }

    for (;;)
    {
        // Block until HINT falling edge fires the notification.
        // Use a timeout so we can also poll for reset events periodically.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        do {
            sh2_service();
        } while (gpio_read_input(&imu.gpio_event) == 0);

        bme280_parse_data(&bme_ctrl);

        // temp_C  = bme_ctrl.data.temperature / 100.0f;
        // press_Pa = bme_ctrl.data.pressure   / 256.0f;
        // hum_pct  = bme_ctrl.data.humidity   / 1024.0f;

        if (!sensors_configured && bno_08x_reset_occurred()) {
            bno_08x_configure_sensors();
            vTaskDelay(pdMS_TO_TICKS(200)); // let a few reports arrive first
            bno_08x_tare();
            sensors_configured = true;
        }
    }
}