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
        
        // Note: The decoded struct uses 'sensorId' instead of 'reportId'
        if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
            
            sh2_RotationVectorWAcc_t *rv = &sensorValue.un.rotationVector;
            
            PRINTF("Q: i:%.2f j:%.2f k:%.2f r:%.2f\r\n", rv->i, rv->j, rv->k, rv->real);
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

    bno_08x_init(&imu, NULL, IMU_Update_Callback, sh2_sensor_callback);

    if (xTaskCreate(SensorTask , "Sensor_task", configMINIMAL_STACK_SIZE + 100, NULL, sensor_task_PRIORITY, &sensorTaskHandle) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for procesing the IMU data.
 */
static void SensorTask(void *pvParameters)
{   

    for (;;)
    {
        // Block indefinitely until the IMU_Update_Callback fires the notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        sh2_service();

    }
}