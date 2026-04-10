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

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Task priorities. */
#define sensor_task_PRIORITY (configMAX_PRIORITIES - 1)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void SensorTask(void *pvParameters);

//void IMU_Callback(LPSPI_Type *base, lpspi_master_edma_handle_t *handle, status_t status, void *userData);

TaskHandle_t sensorTaskHandle = NULL;

/*******************************************************************************
 * Code
 ******************************************************************************/

// void IMU_Callback(LPSPI_Type *base, lpspi_master_edma_handle_t *handle, status_t status, void *userData)
// {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//     if (status == kStatus_Success)
//     {
//         if (sensorTaskHandle != NULL)
//         {
//             // Unblock the SensorTask to let it know the DMA transfer is done
//             vTaskNotifyGiveFromISR(sensorTaskHandle, &xHigherPriorityTaskWoken);
//             portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//         }
//     }
// }

/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitHardware();

    bno_08x_init(false, NULL);

    if (xTaskCreate(SensorTask , "Sensor_task", configMINIMAL_STACK_SIZE + 100, NULL, sensor_task_PRIORITY, NULL) !=
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
 * @brief Task responsible for printing of "Hello world." message.
 */
static void SensorTask(void *pvParameters)
{

    

    for (;;)
    {



    }
}