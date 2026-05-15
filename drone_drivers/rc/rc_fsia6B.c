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
    NVIC_SetPriority(EDMA_0_CH1_IRQn, 4);
    NVIC_SetPriority(EDMA_0_CH2_IRQn, 4);
    uart_get_default_rc_config(&rc_ctrl, func_ptr);
#endif

    uart_init(&rc_ctrl);

    return kRC_StatusSucces;

}

rc_status rc_start_dma_rx(uint8_t *buffer, uint32_t length){
    LPUART_ClearStatusFlags(rc_ctrl.uart_base, kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag | kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag);
    uart_read_dma(&rc_ctrl, buffer, length);
    return kRC_StatusSucces;
}

rc_status rc_sync(uint32_t timeout_ms)
{
    return uart_sync_rx(&rc_ctrl, timeout_ms);
}

rc_status uart_sync_rx(uart_ctrl_t *ctrl, uint32_t timeout_ms) 
{
    LPUART_Type *base = ctrl->uart_base;
    
    // 1. Record the start time
    TickType_t start_ticks = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    
    LPUART_ClearStatusFlags(base, kLPUART_IdleLineFlag | kLPUART_RxOverrunFlag);

    // 2. Wait until the bus goes idle OR the timeout is reached
    while (!(LPUART_GetStatusFlags(base) & kLPUART_IdleLineFlag))
    {
        // Flush the RX register
        if (LPUART_GetStatusFlags(base) & kLPUART_RxDataRegFullFlag)
        {
            (void)LPUART_ReadByte(base);
        }

        // Check if we have exceeded the timeout
        if ((xTaskGetTickCount() - start_ticks) > timeout_ticks)
        {
            // Clear flags before aborting to prevent leaving UART in a bad state
            LPUART_ClearStatusFlags(base, kLPUART_IdleLineFlag | kLPUART_RxOverrunFlag | 
                                          kLPUART_NoiseErrorFlag | kLPUART_FramingErrorFlag | 
                                          kLPUART_ParityErrorFlag);
            return kRC_StatusFail; // Timeout occurred!
        }

        // Yield the CPU for 1 RTOS tick (~1ms) so Motor Tasks can run
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // 3. The bus is now idle. Do one final flush
    while (LPUART_GetStatusFlags(base) & kLPUART_RxDataRegFullFlag)
    {
        (void)LPUART_ReadByte(base);
    }

    // 4. Clear status flags one last time
    LPUART_ClearStatusFlags(base, kLPUART_IdleLineFlag | kLPUART_RxOverrunFlag | 
                                  kLPUART_NoiseErrorFlag | kLPUART_FramingErrorFlag | 
                                  kLPUART_ParityErrorFlag);
                                  
    return kRC_StatusSucces;
}


rc_status rc_parse_frame(uint8_t *buffer, fs_ia6b_frame_t *parsed_frame) {
    
    // 1. Check frame length and command byte
    if (buffer[0] != FS_IA6B_START_BYTE || buffer[1] != 0x40) {
        return kRC_StatusFail;
    }

    // 2. Calculate the 16-bit checksum
    uint16_t calculated_checksum = 0xFFFF;
    for (int i = 0; i < 30; i++) {
        calculated_checksum -= buffer[i];
    }

    // 3. Extract received checksum (iBUS uses little-endian)
    uint16_t received_checksum = buffer[30] | (buffer[31] << 8);

    // 4. Validate checksum
    if (calculated_checksum != received_checksum) {
        return kRC_StatusFail; // CRC Check Failed
    }

    // 5. Populate the parsed_frame struct with valid data
    memcpy(parsed_frame, buffer, sizeof(fs_ia6b_frame_t));

    return kRC_StatusSucces;
}