/*
 * Author: Diego 
 * Created on: 6, April 2026
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
 * @brief Task responsible for parsing the received RC frame, and printing channel values.
 */
static void RCParserTask(void *pvParameters)
{
    fs_ia6b_frame_t current_rc_frame;

    rxOnGoing = true;
    rc_start_dma_rx(g_rxBuffer, FS_IA6B_FRAME_SIZE);

    for (;;)
    {
        /* Wait to be notified by the EDMA ISR */
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == pdTRUE)
        {
            /* Check if the frame passes CRC and is successfully parsed */
            if (rc_parse_frame(g_rxBuffer, &current_rc_frame) == kRC_StatusSucces) 
            {
                PRINTF("Valid Frame! CH1: %d, CH2: %d, CH3: %d, CH4: %d\r\n", 
                        current_rc_frame.channels.CH1.u16, 
                        current_rc_frame.channels.CH2.u16,
                        current_rc_frame.channels.CH3.u16,
                        current_rc_frame.channels.CH4.u16);
            } 
            else 
            {
                //PRINTF("Corrupt Frame or CRC mismatch.\r\n");
                rc_sync();
            }

            /* Clean up and restart DMA reception for the next frame */
            rxBufferEmpty = true;
            rxOnGoing = true;
            
            rc_start_dma_rx(g_rxBuffer, FS_IA6B_FRAME_SIZE);
        }
    }
}