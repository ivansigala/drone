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

/* FreeRTOS — needed for the DMA completion semaphore */
#include "FreeRTOS.h"
#include "semphr.h"

#define IMU_SPI_TRANSFER_BAUDRATE           2000000U  /* 2 MHz */
#define IMU_SPI_MASTER_BASEADDR             (LPSPI3)
#define IMU_SPI_MASTER_INSTANCE             (LPSPI_GetInstance(IMU_SPI_MASTER_BASEADDR))
#define IMU_SPI_MASTER_CLK_FREQ             CLOCK_GetLPFlexCommClkFreq(IMU_SPI_MASTER_INSTANCE)
#define IMU_SPI_MASTER_PCS_FOR_INIT         (kLPSPI_Pcs0)
#define IMU_SPI_MASTER_PCS_FOR_TRANSFER     (kLPSPI_MasterPcs0)
#define IMU_SPI_MASTER_CPOL                 kLPSPI_ClockPolarityActiveLow
#define IMU_SPI_MASTER_CPHA                 kLPSPI_ClockPhaseSecondEdge
#define IMU_SPI_MASTER_DMA_BASE             DMA0
#define IMU_SPI_MASTER_DMA_RX_CHANNEL       7U
#define IMU_SPI_MASTER_DMA_TX_CHANNEL       8U
#define IMU_SPI_TRANSMIT_EDMA_CHANNEL       kDma0RequestMuxLpFlexcomm3Tx
#define IMU_SPI_RECEIVE_EDMA_CHANNEL        kDma0RequestMuxLpFlexcomm3Rx

#define BME_SPI_TRANSFER_BAUDRATE           1000000U  /* 1 MHz */
#define BME_SPI_MASTER_BASEADDR             (LPSPI2)
#define BME_SPI_MASTER_INSTANCE             (LPSPI_GetInstance(BME_SPI_MASTER_BASEADDR))
#define BME_SPI_MASTER_CLK_FREQ             CLOCK_GetLPFlexCommClkFreq(BME_SPI_MASTER_INSTANCE)
#define BME_SPI_MASTER_PCS_FOR_INIT         (kLPSPI_Pcs0)
#define BME_SPI_MASTER_PCS_FOR_TRANSFER     (kLPSPI_MasterPcs0)
#define BME_SPI_MASTER_CPOL                 kLPSPI_ClockPolarityActiveLow
#define BME_SPI_MASTER_CPHA                 kLPSPI_ClockPhaseSecondEdge
#define BME_SPI_MASTER_DMA_BASE             DMA0
#define BME_SPI_MASTER_DMA_RX_CHANNEL       9U
#define BME_SPI_MASTER_DMA_TX_CHANNEL       10U
#define BME_SPI_TRANSMIT_EDMA_CHANNEL       kDma0RequestMuxLpFlexcomm2Tx
#define BME_SPI_RECEIVE_EDMA_CHANNEL        kDma0RequestMuxLpFlexcomm2Rx

#define IMU_SPI_DMA_RX_CH 7U
#define IMU_SPI_DMA_TX_CH 8U

typedef struct spi_ctrl_s{
    LPSPI_Type    *spi_base;
	DMA_Type      *dma_base;
    uint32_t source_clock;
    uint32_t dma_rx_channel;
    uint32_t dma_tx_channel;
    uint32_t   baudrate_u32;
    uint32_t   instance;
    lpspi_clock_phase_t cpha;
    lpspi_clock_polarity_t cpol;
    lpspi_which_pcs_t pcs_for_init;
    lpspi_which_pcs_t pcs_for_transfer;
    lpspi_master_transfer_callback_t spi_callback;
    lpspi_master_edma_transfer_callback_t dma_callback;
    dma_request_source_t edma_rx_channel;
    dma_request_source_t edma_tx_channel;
    bool enable_dma;
    /* Binary semaphore signalled by the DMA completion ISR so that
     * spi_master_transfer() can block the calling task instead of busy-waiting.
     * Created in spi_init() when enable_dma == true.                          */
    SemaphoreHandle_t dma_semaphore;
}spi_ctrl_t;

/*!
 * @brief Gets the default configuration for the IMU SPI master.
 * @param imu Pointer to a spi_ctrl_t struct that will hold the default configuration.
 * @param callback Pointer to the callback function for SPI transfers.
 */
void spi_get_defaultconfig_imu(spi_ctrl_t *imu, void* callback);

/*!
 * @brief Gets the default configuration for the barometer SPI master
 * @param bar Pointer to the spi_ctrl_t structthat will hold the default configuration
 * @param callback Pointer to the callback function for the dma
 */
void spi_get_defaultconfig_bar(spi_ctrl_t *bar, void* callback);

/*!
 * @brief General wraper function to initialize the SPI master.
 * @param spi_master Pointer to a spi_ctrl_t structure that contains the configuration for the SPI master.   
 * @return status_t Returns the status of the SPI initialization.
 */
status_t spi_init(spi_ctrl_t *ctrl);

status_t spi_master_transfer(spi_ctrl_t *ctrl, uint8_t *txData, uint8_t *rxData, size_t dataSize);


#endif /* SPI_DRIVER_MCXN947_H_ */