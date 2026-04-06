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
#define RC_task_PRIORITY 6
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

    rc_init(RC_Callback);

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
            PRINTF("Received frame byte 0: %02X\r\n", g_rxBuffer[0]);

            /* Process your g_rxBuffer data here */
            // rc_parse_frame(...)

            /* Restart DMA reception for the next frame */
            rxBufferEmpty = true;
            rxOnGoing = true;
            rc_start_dma_rx(g_rxBuffer, FS_IA6B_FRAME_SIZE);
        }
    }
}
