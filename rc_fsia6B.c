/*
 * rc_fsia6B.c
 * Author: Diego
 * Created on: 6, April 2026
 */

#include "rc_fsia6B.h"

// Keep static reference to ctrl to use inside start dma function
static uart_ctrl_t rc_ctrl;

rc_status rc_init(void* func_ptr){

#ifdef MCXN947
    rc_ctrl.uart_base = RC_LPUART_BASEADDR;
    rc_ctrl.dma_base = RC_LPUART_DMA_BASEADDR;
    rc_ctrl.dma_rx_channel  = RC_LPUART_RX_DMA_CHANNEL;
    rc_ctrl.edma_rx_channel = RC_LPUART_RX_EDMA_CHANNEL;
    rc_ctrl.callback = (lpuart_edma_transfer_callback_t)func_ptr;
    rc_ctrl.baudrate = RC_UART_BAUDRATE;
    rc_ctrl.enable_dma = true;
#endif

    uart_init(&rc_ctrl);

    return kRC_StatusSucces;

}

rc_status rc_start_dma_rx(uint8_t *buffer, uint32_t length){
    uart_read_dma(&rc_ctrl, buffer, length);
    return kRC_StatusSucces;
}
