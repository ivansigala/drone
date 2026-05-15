/*
 * uart_driver_mcxn947.h	
 * Author: Diego
 * Created on: 6, April 2026
 */


#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_

#include "fsl_lpuart.h"
#include "fsl_lpuart_edma.h"

// ANSI Color Codes (Kept from your file)
//#define ANSI_RESET   "\x1b[0m"
// #define ANSI_RED     "\x1b[31m"
// #define ANSI_GREEN   "\x1b[32m"
// #define ANSI_YELLOW  "\x1b[33m"
// #define ANSI_BLUE    "\x1b[34m"
// #define ANSI_MAGENTA "\x1b[35m"
// #define ANSI_CYAN    "\x1b[36m"
// #define ANSI_WHITE   "\x1b[37m"

// ANSI Style Codes
// #define ANSI_BOLD    "\x1b[1m"
// #define ANSI_CLS     "\x1b[2J\x1b[H"

#define RC_LPUART_RX_EDMA_CHANNEL    kDma0RequestMuxLpFlexcomm1Rx

typedef struct uart_s {
	LPUART_Type  *uart_base;
	DMA_Type     *dma_base;
	dma_request_source_t edma_rx_channel;
	lpuart_edma_transfer_callback_t callback;
	uint32_t baudrate;
	uint32_t dma_rx_channel;
	bool enable_dma;
} uart_ctrl_t;

/* Get Default Config Functions for RC and DShot (ESC) */
void uart_get_default_rc_config(uart_ctrl_t *ctrl, void* callback_func);
void uart_get_default_esc_config(uart_ctrl_t *ctrl, void* callback_func);

void uart_init(uart_ctrl_t *ctrl);
void uart_write(uart_ctrl_t *ctrl, const char* string);
void uart_read_dma(uart_ctrl_t *ctrl, uint8_t *data, uint32_t size);
void uart_abort_rx_dma(uart_ctrl_t *ctrl);
void uart_attach_interrupt(LPUART_Type* uart_base, void* func_ptr);

void UART_HandleIRQ(LPUART_Type* uart_base);

#endif /* UART_DRIVER_H_ */
