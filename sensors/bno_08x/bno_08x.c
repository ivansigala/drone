/*
    bno_08x.c
    Author: Diego
    Created on: 10, April 2026
 */

#include "bno_08x.h"

status_t bno_08x_init(imu_ctrl_t *imu, void* callback)
{
#ifdef MCXN947
    spi_get_defaultconfig_imu(&imu->spi_ctrl, callback);
    imu->spi_ctrl.enable_dma = false;
#endif

    return spi_init(&imu->spi_ctrl);

}

status_t bno_08x_read_reg(imu_ctrl_t *imu, uint8_t reg, uint8_t* data, size_t length)
{
    if (length == 0) return kStatus_InvalidArgument;

    uint8_t tx_buf[2] = {0};

    tx_buf[0] = reg | 0x80; // Set MSB for read operation
    return spi_master_transfer(&imu->spi_ctrl, tx_buf, data, length + 1);
}

status_t bno_08x_write_reg(imu_ctrl_t *imu, uint8_t reg, uint8_t* data, size_t length)
{
    if (length == 0) return kStatus_InvalidArgument;

    uint8_t tx_buf[2] = {0};
    tx_buf[0] = reg & ~0x80; // Clear MSB for write operation
    return spi_master_transfer(&imu->spi_ctrl, tx_buf, data, length + 1);
}