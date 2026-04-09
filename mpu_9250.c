/*
    mpu_9250.c
    Author: Diego
    Created on: 9, April 2026
 */

#include "mpu_9250.h"

static spi_ctrl_t imu;

void mpu9250_init(void* callback){

    spi_ctrl_t imu;
#ifdef MCXN947
    spi_get_defaultconfig_imu(&imu, callback);
#endif

    spi_init(&imu);

}

void mpu9250_read_reg(uint8_t reg, uint8_t* data, size_t length){
    if (length == 0 || length >= MPU9250_MAX_TX_RX_LEN) return;

    uint8_t tx_buf[MPU9250_MAX_TX_RX_LEN] = {0};
    uint8_t rx_buf[MPU9250_MAX_TX_RX_LEN] = {0};

    // Set MSB to 1 for SPI Read
    tx_buf[0] = reg | MPU9250_SPI_READ_BIT; 
    
    // Total transfer length is the 1 address byte + 'length' data bytes
    spi_master_transfer(imu, tx_buf, rx_buf, length + 1);

    // The first byte received is during the address transmission, so skip rx_buf[0]
    memcpy(data, &rx_buf[1], length);
}

void mpu9250_write_reg(uint8_t reg, uint8_t* data, size_t length){
    if (length == 0 || length >= MPU9250_MAX_TX_RX_LEN) return;

    uint8_t tx_buf[MPU9250_MAX_TX_RX_LEN] = {0};

    // Set MSB to 0 for SPI Write
    tx_buf[0] = reg & ~MPU9250_SPI_READ_BIT; 
    
    // Append the payload data to the transmit buffer
    memcpy(&tx_buf[1], data, length);

    // For a write, we don't care about the rx_buf
    spi_master_transfer(imu, tx_buf, NULL, length + 1);
}
