/*
    spi_driver_mcxn947.c
    Author: Diego
    Created on: 9, April 2026
 */

#include "spi_driver_mcxn947.h"


static edma_handle_t g_lpspiRxEdmaHandles[FSL_FEATURE_SOC_LPSPI_COUNT];
static edma_handle_t g_lpspiTxEdmaHandles[FSL_FEATURE_SOC_LPSPI_COUNT];
static lpspi_master_handle_t g_spi_handles[FSL_FEATURE_SOC_LPSPI_COUNT];
AT_NONCACHEABLE_SECTION_INIT( static lpspi_master_edma_handle_t g_spi_edma_handles[FSL_FEATURE_SOC_LPSPI_COUNT]) = {0};


void spi_get_defaultconfig_imu(spi_ctrl_t *imu, void* callback){

    imu->spi_base = IMU_SPI_MASTER_BASEADDR;
    imu->dma_base = IMU_SPI_MASTER_DMA_BASE;
    imu->dma_rx_channel = IMU_SPI_MASTER_DMA_RX_CHANNEL;
    imu->dma_tx_channel = IMU_SPI_MASTER_DMA_TX_CHANNEL;
    imu->baudrate_u32   = IMU_SPI_TRANSFER_BAUDRATE;
    imu->instance       = IMU_SPI_MASTER_INSTANCE;
    imu->pcs_for_init   = IMU_SPI_MASTER_PCS_FOR_INIT;
    imu->pcs_for_transfer = IMU_SPI_MASTER_PCS_FOR_TRANSFER;
    imu->edma_rx_channel  = IMU_SPI_RECEIVE_EDMA_CHANNEL;
    imu->edma_tx_channel  = IMU_SPI_TRANSMIT_EDMA_CHANNEL;
    imu->dma_callback     = (lpspi_master_edma_transfer_callback_t)callback;
    imu->spi_callback     = (lpspi_master_transfer_callback_t)callback;
    imu->enable_dma = true;

}

status_t spi_init(spi_ctrl_t *ctrl){

    uint32_t srcClock_Hz_u32;
    edma_config_t userConfig;
    lpspi_master_config_t masterConfig;

    LPSPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate = ctrl->baudrate_u32;
    masterConfig.whichPcs = ctrl->pcs_for_init;
    masterConfig.cpol     = ctrl->cpol;
    masterConfig.cpha     = ctrl->cpha;
    masterConfig.pcsToSckDelayInNanoSec        = 1000000000U / (masterConfig.baudRate * 2U);
    masterConfig.lastSckToPcsDelayInNanoSec    = 1000000000U / (masterConfig.baudRate * 2U);
    masterConfig.betweenTransferDelayInNanoSec = 1000000000U / (masterConfig.baudRate * 2U);
    
    srcClock_Hz_u32 = IMU_SPI_MASTER_CLK_FREQ;
    LPSPI_MasterInit(ctrl->spi_base, &masterConfig, srcClock_Hz_u32);

    if(ctrl->enable_dma == false){
        LPSPI_MasterTransferCreateHandle(ctrl->spi_base, &(g_spi_handles[ctrl->instance]), ctrl->spi_callback, NULL);
        return kStatus_Success;
    }

    EDMA_GetDefaultConfig(&userConfig);
    EDMA_Init(ctrl->dma_base, &userConfig);

    memset(&(g_lpspiRxEdmaHandles[ctrl->instance]), 0, sizeof(g_lpspiRxEdmaHandles[ctrl->instance]));
    memset(&(g_lpspiTxEdmaHandles[ctrl->instance]), 0, sizeof(g_lpspiTxEdmaHandles[ctrl->instance]));

    EDMA_CreateHandle(&(g_lpspiRxEdmaHandles[ctrl->instance]), ctrl->dma_base,
                      IMU_SPI_DMA_RX_CH);
    EDMA_CreateHandle(&(g_lpspiTxEdmaHandles[ctrl->instance]), ctrl->dma_base,
                      IMU_SPI_DMA_TX_CH);

    LPSPI_MasterTransferCreateHandleEDMA(ctrl->spi_base, &(g_spi_edma_handles[ctrl->instance]), ctrl->dma_callback,
                        NULL, &(g_lpspiRxEdmaHandles[ctrl->instance]),
                        &(g_lpspiTxEdmaHandles[ctrl->instance]));

    return LPSPI_MasterTransferPrepareEDMALite(ctrl->spi_base, &(g_spi_edma_handles[ctrl->instance]), IMU_SPI_MASTER_PCS_FOR_TRANSFER | kLPSPI_MasterByteSwap | kLPSPI_MasterPcsContinuous);

}

status_t spi_master_transfer(spi_ctrl_t *ctrl, uint8_t *txData, uint8_t *rxData, size_t dataSize){

    lpspi_transfer_t masterXfer;


    masterXfer.txData   = txData;
    masterXfer.rxData   = rxData;
    masterXfer.dataSize = dataSize;

    if(ctrl->enable_dma == false){
        return LPSPI_MasterTransferNonBlocking(ctrl->spi_base, &(g_spi_handles[ctrl->instance]), &masterXfer);
    }

    return LPSPI_MasterTransferEDMALite(ctrl->spi_base, &g_spi_edma_handles[ctrl->instance], &masterXfer);

}