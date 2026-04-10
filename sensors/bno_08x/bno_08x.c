/*
    bno_08x.c
    Author: Diego
    Created on: 10, April 2026
 */

#include "bno_08x.h"

status_t bno_08x_init(bool dma_enable, void* callback)
{
    spi_ctrl_t imu;

#ifdef MCXN947
    spi_get_defaultconfig_imu(&imu, callback);
    imu.enable_dma = dma_enable;
#endif

    return spi_init(&imu);

}