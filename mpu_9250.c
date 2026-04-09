/*
    mpu_9250.c
    Author: Diego
    Created on: 9, April 2026
 */

#include "mpu_9250.h"

void mpu9250_init(void* callback){

    spi_ctrl_t imu;
#ifdef MCXN947
    spi_get_defaultconfig_imu(&imu, callback);
#endif

    spi_init(&imu);

}

void mpu9250_read_reg(spi_ctrl_t *imu, uint8_t reg, uint8_t* data, size_t length){
    spi_master_transfer(imu, &reg, data, length);
}

void mpu9250_write_reg(spi_ctrl_t *imu, uint8_t reg, uint8_t* data, size_t length){
    spi_master_transfer(imu, &reg, data, length);
}
