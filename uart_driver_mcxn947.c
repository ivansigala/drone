/*
 * uart_driver_mcxn947.c
 * Author: Diego
 * Created on: 6, April 2026
 */

#include <string.h>
#include "uart_driver_mcxn947.h"

/*******************************************************************************
 * Global Callback Handles
 ******************************************************************************/
void (*UART0_HANDLE)(void) = NULL;
void (*UART1_HANDLE)(void) = NULL;
void (*UART2_HANDLE)(void) = NULL;
void (*UART3_HANDLE)(void) = NULL;
void (*UART4_HANDLE)(void) = NULL;
void (*UART5_HANDLE)(void) = NULL;
void (*UART6_HANDLE)(void) = NULL;
void (*UART7_HANDLE)(void) = NULL;
void (*UART8_HANDLE)(void) = NULL;
void (*UART9_HANDLE)(void) = NULL;

// Storage for the SDK handles (one for each potential UART instance)
static lpuart_handle_t g_lpuartHandles[FSL_FEATURE_SOC_LPUART_COUNT];
static lpuart_edma_handle_t g_lpuartEdmaHandles[FSL_FEATURE_SOC_LPUART_COUNT];
static edma_handle_t g_lpuartRxEdmaHandles[FSL_FEATURE_SOC_LPUART_COUNT];

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

// Helper to convert the Base Address (LPUART4) to the Instance Number (4)
static uint32_t UART_GetInstance(LPUART_Type *base)
{
    if (base == LPUART0) return 0;
    if (base == LPUART1) return 1;
    if (base == LPUART2) return 2;
    if (base == LPUART3) return 3;
    if (base == LPUART4) return 4;
    if (base == LPUART5) return 5;
    if (base == LPUART6) return 6;
    if (base == LPUART7) return 7;
    return 0; // Default/Error
}

void UART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    if (kStatus_LPUART_TxIdle == status)
    {
        // TX Finished
    }
}

/*******************************************************************************
 * Driver Functions
 ******************************************************************************/

void uart_init(uart_ctrl_t *ctrl) {

    uint32_t uart_clk_freq;
    lpuart_config_t config;
    edma_config_t userConfig = {0};
    uint32_t instance = UART_GetInstance(ctrl->uart_base);

    // Get the correct clock frequency
    uart_clk_freq = CLOCK_GetLPFlexCommClkFreq(instance);

    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = ctrl->baudrate;
    config.enableTx     = true;
    config.enableRx     = true;

    LPUART_Init(ctrl->uart_base, &config, uart_clk_freq);

    if(ctrl->enable_dma){
        EDMA_GetDefaultConfig(&userConfig);
        EDMA_Init(ctrl->dma_base, &userConfig);
        EDMA_CreateHandle(&g_lpuartRxEdmaHandles[instance], ctrl->dma_base, ctrl->dma_rx_channel);

        EDMA_SetChannelMux(ctrl->dma_base, ctrl->dma_rx_channel, ctrl->edma_rx_channel);
        
        // Create the EDMA Handle
        LPUART_TransferCreateHandleEDMA(ctrl->uart_base,
                                        &g_lpuartEdmaHandles[instance],
                                        ctrl->callback,
                                        NULL, NULL, &g_lpuartRxEdmaHandles[instance]);
    } else {
        // Create the Transactional Handle
        // This allows the SDK to manage the non-blocking transfer state
        LPUART_TransferCreateHandle(ctrl->uart_base,
                                    &g_lpuartHandles[instance],
                                    (lpuart_transfer_callback_t)ctrl->callback,
                                    NULL);
    }
}

void uart_read_dma(uart_ctrl_t *ctrl, uint8_t *data, uint32_t size) {
    uint32_t instance = UART_GetInstance(ctrl->uart_base);
    lpuart_transfer_t receiveXfer;
    
    receiveXfer.data = data;
    receiveXfer.dataSize = size;
    
    // Clear flags as per the example
    LPUART_ClearStatusFlags(ctrl->uart_base, kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag | kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag);
    LPUART_ReceiveEDMA(ctrl->uart_base, &g_lpuartEdmaHandles[instance], &receiveXfer);
}

void uart_write(uart_ctrl_t *ctrl, const char* string){

    lpuart_transfer_t xfer;
    uint32_t instance = UART_GetInstance(ctrl->uart_base);

    // 1. Fill the Transfer Structure
    xfer.data = (uint8_t*)string;
    xfer.dataSize = strlen(string);

    // 2. Send using the handle
    status_t status;
    do {
        status = LPUART_TransferSendNonBlocking(ctrl->uart_base,
                                                &g_lpuartHandles[instance],
                                                &xfer);
    } while (status == kStatus_LPUART_TxBusy);
}

// ISR Helper to pass control to SDK driver for TX operations
void UART_HandleIRQ(LPUART_Type* uart_base) {
    uint32_t instance = UART_GetInstance(uart_base);
    LPUART_TransferHandleIRQ(instance, &g_lpuartHandles[instance]);
}

void uart_attach_interrupt(LPUART_Type* uart_base, void *func_ptr){

	uint32_t instance = UART_GetInstance(uart_base);
	void (*callback)(void) = (void (*)(void))func_ptr;  /* Properly cast the function pointer */

    // 1. Assign the callback to the correct global variable
	switch (instance) {
		case 0: UART0_HANDLE = callback; EnableIRQ(LP_FLEXCOMM0_IRQn); break;
		case 1: UART1_HANDLE = callback; EnableIRQ(LP_FLEXCOMM1_IRQn); break;
		case 2: UART2_HANDLE = callback; EnableIRQ(LP_FLEXCOMM2_IRQn); break;
		case 3: UART3_HANDLE = callback; EnableIRQ(LP_FLEXCOMM3_IRQn); break;
		case 4: UART4_HANDLE = callback; EnableIRQ(LP_FLEXCOMM4_IRQn); break;
		case 5: UART5_HANDLE = callback; EnableIRQ(LP_FLEXCOMM5_IRQn); break;
		case 6: UART6_HANDLE = callback; EnableIRQ(LP_FLEXCOMM6_IRQn); break;
		case 7: UART7_HANDLE = callback; EnableIRQ(LP_FLEXCOMM7_IRQn); break;
        default: break; // Invalid instance
    }
    // 2. Enable the RX Interrupt on the hardware
    LPUART_EnableInterrupts(uart_base, kLPUART_RxDataRegFullInterruptEnable);
}

/*******************************************************************************
 * Interrupt Service Routines (ISRs)
 ******************************************************************************/

void LP_FLEXCOMM0_IRQHandler(void){
	if(UART0_HANDLE != NULL) UART0_HANDLE();
	UART_HandleIRQ(LPUART0);
	SDK_ISR_EXIT_BARRIER;
}

void LP_FLEXCOMM1_IRQHandler(void){
	/* For LPUART1 with custom RX callback, only call the user callback
	   DO NOT call UART_HandleIRQ since we're not using the SDK transfer API */
	if(UART1_HANDLE != NULL) UART1_HANDLE();
	SDK_ISR_EXIT_BARRIER;
}

void LP_FLEXCOMM2_IRQHandler(void){
	if(UART2_HANDLE != NULL) UART2_HANDLE();
	UART_HandleIRQ(LPUART2);
	SDK_ISR_EXIT_BARRIER;
}

void LP_FLEXCOMM3_IRQHandler(void){
	if(UART3_HANDLE != NULL) UART3_HANDLE();
	UART_HandleIRQ(LPUART3);
	SDK_ISR_EXIT_BARRIER;
}

void LP_FLEXCOMM4_IRQHandler(void){
	if(UART4_HANDLE != NULL) UART4_HANDLE();
	UART_HandleIRQ(LPUART4);
	SDK_ISR_EXIT_BARRIER;
}

void LP_FLEXCOMM5_IRQHandler(void){
	if(UART5_HANDLE != NULL) UART5_HANDLE();
	UART_HandleIRQ(LPUART5);
	SDK_ISR_EXIT_BARRIER;
}

void LP_FLEXCOMM6_IRQHandler(void){
	if(UART6_HANDLE != NULL) UART6_HANDLE();
	UART_HandleIRQ(LPUART6);
	SDK_ISR_EXIT_BARRIER;
}

void LP_FLEXCOMM7_IRQHandler(void){
	if(UART7_HANDLE != NULL) UART7_HANDLE();
	UART_HandleIRQ(LPUART7);
	SDK_ISR_EXIT_BARRIER;
}