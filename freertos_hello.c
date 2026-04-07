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

/* user includes. */
#include "rc_fsia6B.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Task priorities. */
#define RC_task_PRIORITY 4
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void RCParserTask(void *pvParameters);
void RC_Callback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/
// lpuart_edma_handle_t g_lpuartEdmaHandle;
AT_NONCACHEABLE_SECTION_INIT(uint8_t g_rxBuffer[FS_IA6B_FRAME_SIZE]) = {0};
volatile bool rxBufferEmpty                                          = true;
volatile bool rxOnGoing                                              = false;

/* Task handle for notifications */
TaskHandle_t rcParserTaskHandle = NULL;

/* Queue for successfully parsed channels */
QueueHandle_t rcChannelQueue = NULL;

/*******************************************************************************
 * Code
 ******************************************************************************/

void RC_Callback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (kStatus_LPUART_RxIdle == status)
    {
        rxBufferEmpty = false;
        rxOnGoing     = false;

        /* Notify the parsing task that data is ready */
        vTaskNotifyGiveFromISR(rcParserTaskHandle, &xHigherPriorityTaskWoken);
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitHardware();

    /* FreeRTOS interrupt priority fix */
    NVIC_SetPriority(EDMA_0_CH1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);

    rc_init(RC_Callback);

    /* Create Queue to hold the parsed channels data (buffer size of 5 frames) */
    rcChannelQueue = xQueueCreate(5, sizeof(fs_ia6b_channels_t));
    if (rcChannelQueue == NULL)
    {
        PRINTF("Queue creation failed!.\r\n");
        while (1);
    }

    if (xTaskCreate(RCParserTask, "rc_task", configMINIMAL_STACK_SIZE + 100, NULL, RC_task_PRIORITY, &rcParserTaskHandle) !=
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
static void RCParserTask(void *pvParameters)
{
    // Start initial DMA reception
    rxOnGoing = true;
    rc_start_dma_rx(g_rxBuffer, FS_IA6B_FRAME_SIZE);

    for (;;)
    {
        /* Wait to be notified by the EDMA ISR */
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == pdTRUE)
        {
            /* Data has been received via DMA */
            PRINTF("Received frame byte 0: %02X\r\n", g_rxBuffer[31]);

            // /* Process your g_rxBuffer data here */
            fs_ia6b_channels_t extracted_channels;
            rc_parse_frame((const uint8_t*)g_rxBuffer, &extracted_channels);
            //if (rc_parse_frame((const uint8_t*)g_rxBuffer, &extracted_channels) == kRC_StatusSucces)
            // {
            //     /* Valid frame decoded. Add the struct onto the parsed frames queue. */
            //     if (xQueueSend(rcChannelQueue, &extracted_channels, portMAX_DELAY) == pdPASS)
            //     {
            //         // Frame parsed and enqueued correctly
            //         PRINTF("Valid Frame: CH1=%d CH2=%d\r\n", 
            //             extracted_channels.CH1.u16, extracted_channels.CH2.u16);
            //     }
            // } else {
            //     PRINTF("Invalid/bad CRC frame ignored...\r\n");
            // }

            /* Restart DMA reception for the next frame */
            rxBufferEmpty = true;
            rxOnGoing = true;
            rc_start_dma_rx(g_rxBuffer, FS_IA6B_FRAME_SIZE);
        }
    }
}
