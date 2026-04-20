/*
    bme280.h
    Author: Diego
    Created on: 13, April 2026
 */


#ifndef BME280_H_
#define BME280_H_


#include "spi_driver_mcxn947.h"

typedef struct bme280_calib_data_s
{
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;

} bme280_calib_data_t;

typedef struct bme280_data_s
{
    int32_t temperature;
    int64_t pressure;
    float humidity;
} bme280_data_t;

typedef struct bme280_ctrl_s
{
    spi_ctrl_t spi_ctrl;
    bme280_data_t data;
    bme280_calib_data_t cal_data;

} bme280_ctrl_t;

status_t bme280_init(bme280_ctrl_t *bme_ctl, void* spi_callback);




#endif /* BME280_H_ */