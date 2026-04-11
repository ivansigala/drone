/*
    bno_08x.h
    Author: Diego
    Created on: 10, April 2026
 */

#ifndef BNO_08X_H_
#define BNO_08X_H_

#ifdef MCXN947
#include "spi_driver_mcxn947.h"
#include "gpio_driver_mcxn947.h"
#endif

typedef struct imu_ctrl_s
{
    spi_ctrl_t spi_ctrl;

} imu_ctrl_t;

status_t bno_08x_init(imu_ctrl_t *imu, void* spi_callback, void* gpio_callback);
status_t bno_08x_read_reg(imu_ctrl_t *imu, uint8_t reg, uint8_t* data, size_t length);
status_t bno_08x_write_reg(imu_ctrl_t *imu, uint8_t reg, uint8_t* data, size_t length);

#endif /* BNO_08X_H_ */