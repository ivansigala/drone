/*
    i2c_driver_mcxn947.c
    Author: Diego
    Created on: 5, May 2026
 */

#include "i2c_driver_mcxn947.h"

#include <string.h>


/* Per-LPI2C-instance handles.  The eDMA handles must be in normal RAM (the
 * SDK touches their fields from ISR context), but the master EDMA handle
 * holds the active transfer descriptor and is safest in the same non-
 * cacheable region the SPI wrapper uses for its mirror.                      */
static edma_handle_t g_lpi2cRxEdmaHandles[FSL_FEATURE_SOC_LPI2C_COUNT];
static edma_handle_t g_lpi2cTxEdmaHandles[FSL_FEATURE_SOC_LPI2C_COUNT];
AT_NONCACHEABLE_SECTION_INIT(static lpi2c_master_edma_handle_t g_i2c_edma_handles[FSL_FEATURE_SOC_LPI2C_COUNT]) = {0};


/*
 * Internal DMA-completion callback.
 *
 * Called from the eDMA IRQ handler when the LPI2C DMA transfer finishes.
 * Gives the per-instance binary semaphore so that i2c_master_*() can
 * unblock the calling FreeRTOS task instead of busy-polling.
 *
 * userData is the SemaphoreHandle_t stored in i2c_ctrl_t.dma_semaphore.
 */
static void i2c_dma_complete_callback(LPI2C_Type *base,
                                      lpi2c_master_edma_handle_t *handle,
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


void i2c_get_defaultconfig_tof(i2c_ctrl_t *tof)
{
    tof->i2c_base        = TOF_I2C_BASEADDR;
    tof->dma_base        = TOF_I2C_DMA_BASE;
    tof->dma_rx_channel  = TOF_I2C_DMA_RX_CHANNEL;
    tof->dma_tx_channel  = TOF_I2C_DMA_TX_CHANNEL;
    tof->baudrate        = TOF_I2C_BAUDRATE;
    tof->instance        = TOF_I2C_INSTANCE;
    tof->edma_rx_channel = TOF_I2C_RX_EDMA_REQUEST;
    tof->edma_tx_channel = TOF_I2C_TX_EDMA_REQUEST;
    tof->source_clock    = TOF_I2C_CLK_FREQ;
    tof->enable_dma      = true;
}


status_t i2c_init(i2c_ctrl_t *ctrl)
{
    lpi2c_master_config_t masterConfig;
    LPI2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz = ctrl->baudrate;

    LPI2C_MasterInit(ctrl->i2c_base, &masterConfig, ctrl->source_clock);

    if (!ctrl->enable_dma)
    {
        /* Pure-blocking path — caller will use LPI2C_MasterTransferBlocking
         * indirectly through i2c_*().  No handle / semaphore needed.       */
        return kStatus_Success;
    }

    /* NOTE: EDMA_Init is intentionally NOT called here — see header.        */

    /* Route the LPI2C TX/RX DMA requests to the chosen eDMA channels.       */
    EDMA_SetChannelMux(ctrl->dma_base, ctrl->dma_rx_channel, ctrl->edma_rx_channel);
    EDMA_SetChannelMux(ctrl->dma_base, ctrl->dma_tx_channel, ctrl->edma_tx_channel);

    memset(&g_lpi2cRxEdmaHandles[ctrl->instance], 0,
           sizeof(g_lpi2cRxEdmaHandles[ctrl->instance]));
    memset(&g_lpi2cTxEdmaHandles[ctrl->instance], 0,
           sizeof(g_lpi2cTxEdmaHandles[ctrl->instance]));

    EDMA_CreateHandle(&g_lpi2cRxEdmaHandles[ctrl->instance],
                      ctrl->dma_base, ctrl->dma_rx_channel);
    EDMA_CreateHandle(&g_lpi2cTxEdmaHandles[ctrl->instance],
                      ctrl->dma_base, ctrl->dma_tx_channel);

    /* Binary semaphore: the DMA-completion ISR gives, the task takes.       */
    ctrl->dma_semaphore = xSemaphoreCreateBinary();
    if (ctrl->dma_semaphore == NULL)
    {
        return kStatus_Fail;
    }

    LPI2C_MasterCreateEDMAHandle(ctrl->i2c_base,
                                 &g_i2c_edma_handles[ctrl->instance],
                                 &g_lpi2cRxEdmaHandles[ctrl->instance],
                                 &g_lpi2cTxEdmaHandles[ctrl->instance],
                                 i2c_dma_complete_callback,
                                 ctrl->dma_semaphore);

    return kStatus_Success;
}


/* Single point of truth for "fire transfer + wait for it" so the
 * register-style helpers below stay tiny.                                   */
static status_t i2c_do_transfer(i2c_ctrl_t *ctrl, lpi2c_master_transfer_t *xfer)
{
    if (!ctrl->enable_dma)
    {
        return LPI2C_MasterTransferBlocking(ctrl->i2c_base, xfer);
    }

    status_t st = LPI2C_MasterTransferEDMA(ctrl->i2c_base,
                                           &g_i2c_edma_handles[ctrl->instance],
                                           xfer);
    if (st != kStatus_Success)
    {
        return st;
    }

    if (xSemaphoreTake(ctrl->dma_semaphore, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        /* Timeout — abort so the LPI2C peripheral isn't left mid-transfer.  */
        LPI2C_MasterTransferAbortEDMA(ctrl->i2c_base,
                                      &g_i2c_edma_handles[ctrl->instance]);
        return kStatus_Timeout;
    }
    return kStatus_Success;
}


status_t i2c_read_reg(i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg,
                      uint8_t *data, size_t size)
{
    lpi2c_master_transfer_t xfer = {0};
    xfer.slaveAddress   = addr7;
    xfer.direction      = kLPI2C_Read;
    xfer.subaddress     = reg;
    xfer.subaddressSize = 1U;
    xfer.data           = data;
    xfer.dataSize       = size;
    xfer.flags          = kLPI2C_TransferDefaultFlag;
    return i2c_do_transfer(ctrl, &xfer);
}


status_t i2c_write_reg(i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg,
                       const uint8_t *data, size_t size)
{
    lpi2c_master_transfer_t xfer = {0};
    xfer.slaveAddress   = addr7;
    xfer.direction      = kLPI2C_Write;
    xfer.subaddress     = reg;
    xfer.subaddressSize = 1U;
    xfer.data           = (void *)data;
    xfer.dataSize       = size;
    xfer.flags          = kLPI2C_TransferDefaultFlag;
    return i2c_do_transfer(ctrl, &xfer);
}


status_t i2c_read_u8(i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg, uint8_t *out)
{
    return i2c_read_reg(ctrl, addr7, reg, out, 1U);
}


status_t i2c_read_u16(i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg, uint16_t *out)
{
    uint8_t buf[2] = {0};
    status_t st = i2c_read_reg(ctrl, addr7, reg, buf, 2U);
    if (st == kStatus_Success)
    {
        /* VL53L0X / most sensors are big-endian on the wire (datasheet §4.2). */
        *out = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    }
    return st;
}


status_t i2c_write_u8(i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg, uint8_t val)
{
    return i2c_write_reg(ctrl, addr7, reg, &val, 1U);
}


status_t i2c_write_u16(i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg, uint16_t val)
{
    uint8_t buf[2] = {
        (uint8_t)((val >> 8) & 0xFFU),
        (uint8_t)( val       & 0xFFU)
    };
    return i2c_write_reg(ctrl, addr7, reg, buf, 2U);
}
