/*
 * dma.c
 *
 *  Created on: Mar 10, 2026
 *      Author: diego
 */


#include "dma_driver_mcxn947.h"

/*******************************************************************************
 * Chip Specific Defines (Moved from DShot header)
 ******************************************************************************/
#define DSHOT_DMA_BASE        DMA0

/* Motor Hardware Mapping (DMA) */
#define M0_DMA_CH             3U
#define M0_DMA_REQ            kDma0RequestMuxFlexPwm1ReqVal0

#define M1_DMA_CH             4U
#define M1_DMA_REQ            kDma0RequestMuxFlexPwm1ReqVal1

#define M2_DMA_CH             5U
#define M2_DMA_REQ            kDma0RequestMuxFlexPwm1ReqVal2

#define M3_DMA_CH             6U
#define M3_DMA_REQ            kDma0RequestMuxFlexPwm1ReqVal3

// The handles are now private to this file
static edma_handle_t s_dma_handles[MAX_DMA_CHANNELS];

void dma_get_default_dshot_config(dma_ctrl_t *ctrl, uint8_t motor_id) {
    ctrl->dma_base = DSHOT_DMA_BASE;
    switch (motor_id) {
        case 0:
            ctrl->channel_u16 = M0_DMA_CH;
            ctrl->dma_request = M0_DMA_REQ;
            break;
        case 1:
            ctrl->channel_u16 = M1_DMA_CH;
            ctrl->dma_request = M1_DMA_REQ;
            break;
        case 2:
            ctrl->channel_u16 = M2_DMA_CH;
            ctrl->dma_request = M2_DMA_REQ;
            break;
        case 3:
            ctrl->channel_u16 = M3_DMA_CH;
            ctrl->dma_request = M3_DMA_REQ;
            break;
        default:
            break;
    }
}

void dma_init_channel(dma_ctrl_t *dma) {

    edma_config_t edmaConfig;

    EDMA_GetDefaultConfig(&edmaConfig);
    EDMA_Init(dma->dma_base, &edmaConfig);

    EDMA_CreateHandle(&s_dma_handles[dma->channel_u16], dma->dma_base, dma->channel_u16);

    EDMA_SetChannelMux(dma->dma_base, dma->channel_u16, dma->dma_request);

}


void dma_transfer_submit(uint32_t channel, uint32_t srcAddr, uint32_t destAddr, uint32_t itemSize, uint32_t totalBytes) {

	edma_transfer_config_t transferConfig;

    EDMA_PrepareTransfer(&transferConfig,
                         (void *)srcAddr, itemSize,
                         (void *)destAddr, itemSize,
                         itemSize,       // Bytes per request
                         totalBytes,     // Total bytes for the frame
                         kEDMA_MemoryToPeripheral);

    EDMA_SubmitTransfer(&s_dma_handles[channel], &transferConfig);
    EDMA_StartTransfer(&s_dma_handles[channel]);
}


void dma_transfer_submit_channels(uint32_t *channels, uint32_t *srcAddr, uint32_t *destAddr, uint32_t itemSize, uint32_t totalBytes, uint32_t numChannels) {

	edma_transfer_config_t transferConfig;

    for(int i = 0; i < numChannels; i++) {
        EDMA_PrepareTransfer(&transferConfig,
                            (void *)srcAddr[i], itemSize,
                            (void *)destAddr[i], itemSize,
                            itemSize,       // Bytes per request
                            totalBytes,     // Total bytes for the frame
                            kEDMA_MemoryToPeripheral);
        EDMA_SubmitTransfer(&s_dma_handles[channels[i]], &transferConfig);
    }

    __disable_irq();
    for (uint32_t i = 0; i < numChannels; i++) {
        EDMA_StartTransfer(&s_dma_handles[channels[i]]);
    }
    __enable_irq();
    
}
