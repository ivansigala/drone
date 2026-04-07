/*
 * dma.h
 *
 *  Created on: Mar 10, 2026
 *      Author: diego
 */

#ifndef DMA_DRIVER_MCXN947_H_
#define DMA_DRIVER_MCXN947_H_


#include "fsl_edma.h"

#define MAX_DMA_CHANNELS 8

typedef struct dma_s {
	DMA_Type  *dma_base;
	uint32_t channel_u16;
	dma_request_source_t dma_request;
} dma_ctrl_t;

/* Get Default DMA Config for DShot Motors */
void dma_get_default_dshot_config(dma_ctrl_t *ctrl, uint8_t motor_id);

void dma_init_channel(dma_ctrl_t *config);

void dma_transfer_submit(uint32_t channel, uint32_t srcAddr, uint32_t destAddr, uint32_t itemSize, uint32_t totalBytes);



#endif /* DMA_DRIVER_MCXN947_H_ */
