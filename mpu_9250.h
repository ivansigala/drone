/*
    mpu_9250.h
    Author: Diego
    Created on: 9, April 2026
 */

#ifndef MPU_9250_H_
#define MPU_9250_H_

#ifdef MCXN947
#include "spi_driver_mcxn947.h"
#include "dma_driver_mcxn947.h"
#endif

#include "mpu_9250_reg.h"

#define MPU9250_SPI_READ_BIT  0x80
#define MPU9250_MAX_TX_RX_LEN 32

void mpu9250_init(void* callback);
void mpu9250_read_reg(uint8_t reg, uint8_t* data, size_t length);
void mpu9250_write_reg(uint8_t reg, uint8_t* data, size_t length);


#endif /* MPU_9250_H_ */