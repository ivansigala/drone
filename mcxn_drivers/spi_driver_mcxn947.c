/*
    spi_driver_mcxn947.c
    Author: Diego
    Created on: 9, April 2026
 */

#include "spi_driver_mcxn947.h"

/* FreeRTOS — needed for the DMA completion semaphore */
#include "FreeRTOS.h"
#include "semphr.h"


static edma_handle_t g_lpspiRxEdmaHandles[FSL_FEATURE_SOC_LPSPI_COUNT];
static edma_handle_t g_lpspiTxEdmaHandles[FSL_FEATURE_SOC_LPSPI_COUNT];
static lpspi_master_handle_t g_spi_handles[FSL_FEATURE_SOC_LPSPI_COUNT];
AT_NONCACHEABLE_SECTION_INIT( static lpspi_master_edma_handle_t g_spi_edma_handles[FSL_FEATURE_SOC_LPSPI_COUNT]) = {0};

/*
 * Internal DMA completion callback.
 *
 * Called from the eDMA IRQ handler when the LPSPI DMA transfer finishes.
 * Gives the per-instance binary semaphore so that spi_master_transfer()
 * can unblock the calling FreeRTOS task instead of busy-polling.
 *
 * userData is the SemaphoreHandle_t stored in spi_ctrl_t.dma_semaphore and
 * installed as the userData argument in LPSPI_MasterTransferCreateHandleEDMA.
 */
static void spi_dma_complete_callback(LPSPI_Type *base,
                                      lpspi_master_edma_handle_t *handle,
                                      status_t completionStatus,
                                      void *userData)
{
    (void)base;
    (void)handle;
    (void)completionStatus;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)userData, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


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
    imu->cpol             = IMU_SPI_MASTER_CPOL;
    imu->cpha             = IMU_SPI_MASTER_CPHA;
    imu->enable_dma = true;   // use eDMA transfers; semaphore created in spi_init()

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

    /* Route the LPSPI RX and TX DMA request sources to the chosen eDMA channels.
     * Without this the LPSPI peripheral cannot trigger a DMA transfer.
     * NOTE: if your SDK spells this EDMA4_SetChannelMux, rename accordingly.   */
    EDMA_SetChannelMux(ctrl->dma_base, IMU_SPI_DMA_RX_CH, ctrl->edma_rx_channel);
    EDMA_SetChannelMux(ctrl->dma_base, IMU_SPI_DMA_TX_CH, ctrl->edma_tx_channel);

    memset(&(g_lpspiRxEdmaHandles[ctrl->instance]), 0, sizeof(g_lpspiRxEdmaHandles[ctrl->instance]));
    memset(&(g_lpspiTxEdmaHandles[ctrl->instance]), 0, sizeof(g_lpspiTxEdmaHandles[ctrl->instance]));

    EDMA_CreateHandle(&(g_lpspiRxEdmaHandles[ctrl->instance]), ctrl->dma_base,
                      IMU_SPI_DMA_RX_CH);
    EDMA_CreateHandle(&(g_lpspiTxEdmaHandles[ctrl->instance]), ctrl->dma_base,
                      IMU_SPI_DMA_TX_CH);

    /* Create the binary semaphore used to signal completion from the DMA ISR.
     * Must be created before the EDMA handle so userData is valid.            */
    ctrl->dma_semaphore = xSemaphoreCreateBinary();
    if (ctrl->dma_semaphore == NULL)
    {
        return kStatus_Fail;
    }

    /* Install the internal callback; pass the semaphore as userData so the
     * ISR can give it without needing any global state.                        */
    LPSPI_MasterTransferCreateHandleEDMA(ctrl->spi_base,
                        &(g_spi_edma_handles[ctrl->instance]),
                        spi_dma_complete_callback,
                        ctrl->dma_semaphore,
                        &(g_lpspiRxEdmaHandles[ctrl->instance]),
                        &(g_lpspiTxEdmaHandles[ctrl->instance]));

    /* kLPSPI_MasterPcsContinuous keeps CS low for the entire multi-byte packet.
     * This MUST match the flag used in the blocking path.                      */
    return LPSPI_MasterTransferPrepareEDMALite(ctrl->spi_base,
                        &(g_spi_edma_handles[ctrl->instance]),
                        IMU_SPI_MASTER_PCS_FOR_TRANSFER |
                        kLPSPI_MasterByteSwap           |
                        kLPSPI_MasterPcsContinuous);

}

status_t spi_master_transfer(spi_ctrl_t *ctrl, uint8_t *txData, uint8_t *rxData, size_t dataSize){

    lpspi_transfer_t masterXfer;


    masterXfer.txData   = txData;
    masterXfer.rxData   = rxData;
    masterXfer.dataSize = dataSize;

    masterXfer.configFlags = ctrl->pcs_for_transfer | kLPSPI_MasterByteSwap | kLPSPI_MasterPcsContinuous;

    if(ctrl->enable_dma == false){
        return LPSPI_MasterTransferBlocking(ctrl->spi_base, &masterXfer);
    }

    /* Start the non-blocking DMA transfer, then park the calling task on the
     * semaphore.  spi_dma_complete_callback() gives the semaphore from the
     * eDMA IRQ so the task wakes up exactly when the transfer is done.
     * A 100 ms timeout guards against a lost interrupt or hardware hang.      */
    status_t status = LPSPI_MasterTransferEDMALite(ctrl->spi_base,
                                                   &g_spi_edma_handles[ctrl->instance],
                                                   &masterXfer);
    if (status != kStatus_Success)
    {
        return status;
    }

    if (xSemaphoreTake(ctrl->dma_semaphore, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        /* Timeout — abort the transfer so the peripheral is left in a clean state */
        LPSPI_MasterTransferAbortEDMA(ctrl->spi_base, &g_spi_edma_handles[ctrl->instance]);
        return kStatus_Timeout;
    }

    return kStatus_Success;
}
