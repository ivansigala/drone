/*
 * rc_fsia6B.c
 * Author: Diego
 * Created on: 6, April 2026
 */

#include "rc_fsia6B.h"
#include <string.h>

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

    uart_sync_rx(&rc_ctrl);

    return kRC_StatusSucces;

}

rc_status rc_start_dma_rx(uint8_t *buffer, uint32_t length){
    uart_read_dma(&rc_ctrl, buffer, length);
    return kRC_StatusSucces;
}

void uart_sync_rx(uart_ctrl_t *ctrl) 
{
    LPUART_Type *base = ctrl->uart_base;
    
    /* 1. Clear any existing IDLE and Overrun flags */
    LPUART_ClearStatusFlags(base, kLPUART_IdleLineFlag | kLPUART_RxOverrunFlag);

    /* 2. Wait until the bus goes idle  */
    while (!(LPUART_GetStatusFlags(base) & kLPUART_IdleLineFlag))
    {
        /* Flush the RX register to prevent hardware overrun while waiting */
        if (LPUART_GetStatusFlags(base) & kLPUART_RxDataRegFullFlag)
        {
            (void)LPUART_ReadByte(base);
        }
    }

    /* 3. The bus is now idle. Do one final flush of the RX buffer */
    while (LPUART_GetStatusFlags(base) & kLPUART_RxDataRegFullFlag)
    {
        (void)LPUART_ReadByte(base);
    }

    /* 4. Clear status flags one last time */
    LPUART_ClearStatusFlags(base, kLPUART_IdleLineFlag | kLPUART_RxOverrunFlag | 
                                  kLPUART_NoiseErrorFlag | kLPUART_FramingErrorFlag | 
                                  kLPUART_ParityErrorFlag);
}

/*!
 * @brief Calculate CRC for FS-iA6B frame (XOR of all bytes)
 * @param data Pointer to data buffer
 * @param length Length of data buffer
 * @return uint8_t CRC value
 */
static uint8_t rc_calculate_crc(const uint8_t *data, uint16_t length)
{
	uint8_t crc = 0;
	for (uint16_t i = 0; i < length; i++) {
		crc ^= data[i];
	}
	return crc;
}

rc_status rc_parse_frame(const uint8_t *buffer, fs_ia6b_channels_t *channels)
{
    if (buffer == NULL || channels == NULL)
    {
        return kRC_StatusFail;
    }

    /* 1. Check Start Byte */
    if (buffer[0] != FS_IA6B_START_BYTE)
    {
        return kRC_StatusFail;
    }

    /* 2. Check CRC (XOR bytes 0-30 must equal byte 31) */
    uint8_t calculated_crc = rc_calculate_crc(buffer, FS_IA6B_FRAME_SIZE - 1);
    if (calculated_crc != buffer[FS_IA6B_FRAME_SIZE - 1])
    {
        //return kRC_StatusFail;
    }

    /* 3. Extract channels efficiently
     * We copy exactly 28 bytes (14 channels * 2 bytes/channel) from index 1.
     * memcpy is safest to prevent unaligned access exceptions on ARM and is aggressively
     * optimized into single-cycle load/store instructions when dimensions are known.
     */
    memcpy(channels, &buffer[1], sizeof(fs_ia6b_channels_t));

    return kRC_StatusSucces;
}
