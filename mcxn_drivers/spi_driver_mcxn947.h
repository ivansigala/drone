/*
    spi_driver_mcxn947.h
    Author: Diego
    Created on: 9, April 2026
 */

#ifndef SPI_DRIVER_MCXN947_H_
#define SPI_DRIVER_MCXN947_H_

#include "fsl_device_registers.h"
#include "fsl_lpspi.h"
#include "fsl_lpspi_edma.h"

#define IMU_SPI_TRANSFER_BAUDRATE           1000000U  /* 1 MHz */
#define IMU_SPI_MASTER_BASEADDR             (LPSPI3)
#define IMU_SPI_MASTER_INSTANCE             (LPSPI_GetInstance(IMU_SPI_MASTER_BASEADDR))
#define IMU_SPI_MASTER_CLK_FREQ             CLOCK_GetFreq(IMU_SPI_MASTER_INSTANCE)
#define IMU_SPI_MASTER_PCS_FOR_INIT         (kLPSPI_Pcs0)
#define IMU_SPI_MASTER_PCS_FOR_TRANSFER     (kLPSPI_MasterPcs0)
#define IMU_SPI_MASTER_DMA_BASE             DMA0
#define IMU_SPI_MASTER_DMA_RX_CHANNEL       0U
#define IMU_SPI_MASTER_DMA_TX_CHANNEL       1U
#define IMU_SPI_TRANSMIT_EDMA_CHANNEL       kDma0RequestMuxLpFlexcomm3Tx
#define IMU_SPI_RECEIVE_EDMA_CHANNEL        kDma0RequestMuxLpFlexcomm3Rx

#define IMU_SPI_DMA_RX_CH 7U
#define IMU_SPI_DMA_TX_CH 8U

typedef struct spi_ctrl_s{
    LPSPI_Type    *spi_base;
	DMA_Type      *dma_base;
    uint32_t dma_rx_channel;
    uint32_t dma_tx_channel;
    uint32_t   baudrate_u32;
    uint32_t   instance;
    lpspi_which_pcs_t pcs_for_init;
    lpspi_which_pcs_t pcs_for_transfer;
    lpspi_master_edma_transfer_callback_t callback;
    dma_request_source_t edma_rx_channel;
    dma_request_source_t edma_tx_channel;
    bool enable_dma;
}spi_ctrl_t;

void spi_get_defaultconfig_imu(spi_ctrl_t *imu);

/*!
 * @brief General wraper function to initialize the SPI master.
 * @param spi_master Pointer to a spi_ctrl_t structure that contains the configuration for the SPI master.   
 */
void spi_init(spi_ctrl_t *spi_master);

status_t spi_master_transfer(spi_ctrl_t *ctrl, uint8_t *txData, uint8_t *rxData, size_t dataSize);


#endif /* SPI_DRIVER_MCXN947_H_ */