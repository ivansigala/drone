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
#include "mpu_9250.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Task priorities. */
#define sensor_task_PRIORITY (configMAX_PRIORITIES - 1)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void SensorTask(void *pvParameters);

void IMU_Callback(LPSPI_Type *base, lpspi_master_edma_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Code
 ******************************************************************************/

void IMU_Callback(LPSPI_Type *base, lpspi_master_edma_handle_t *handle, status_t status, void *userData)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (status == kStatus_Success)
    {
        if (sensorTaskHandle != NULL)
        {
            // Unblock the SensorTask to let it know the DMA transfer is done
            vTaskNotifyGiveFromISR(sensorTaskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

    mpu9250_init(IMU_Callback);

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
    // --------------------------------------------------------------
    // PHASE 1: MPU-9250 Register Configuration
    // --------------------------------------------------------------
    uint8_t cfg_data = 0;

    // Example: Wake up the MPU9250 (Clear sleep bit in PWR_MGMT_1)
    cfg_data = 0x00;
    mpu9250_write_reg(MPU9250_PWR_MGMT_1, &cfg_data, 1);
    // Wait for DMA write to complete
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow oscillator to stabilize

    // Example: Enable Data Ready Interrupt on the IMU (INT_ENABLE register)
    cfg_data = 0x01; // RAW_RDY_EN bit
    mpu9250_write_reg(MPU9250_INT_ENABLE, &cfg_data, 1);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // (Add your other configurations here: Gyro scale, Accel scale, DLPF, etc.)

    // --------------------------------------------------------------
    // PHASE 2: Main Control Loop
    // --------------------------------------------------------------
    uint8_t rx_buffer[14]; // 6 bytes Accel + 2 bytes Temp + 6 bytes Gyro

    for (;;)
    {
        // 1. Wait for the hardware INT pin (GPIO ISR) to notify us that data is ready
        //    This effectively sets your task frequency to the IMU's sample rate!
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 2. Start the non-blocking SPI DMA read for all 14 data registers
        //    (Starts at ACCEL_XOUT_H 0x3B)
        mpu9250_read_reg(MPU9250_ACCEL_XOUT_H, rx_buffer, 14);

        // 3. Wait for the SPI DMA Transfer to complete (IMU_Callback)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 4. Data is now safely in rx_buffer. Parse it.
        int16_t accel_x = (rx_buffer[0] << 8) | rx_buffer[1];
        int16_t accel_y = (rx_buffer[2] << 8) | rx_buffer[3];
    }
}