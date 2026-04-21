/*
    bme280.h
    Author: Diego
    Created on: 13, April 2026
 */


#ifndef BME280_H_
#define BME280_H_

#ifdef MCXN947
#include "spi_driver_mcxn947.h"
#endif

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
    int32_t  temperature;   /* 0.01 °C units  — e.g. 5123 → 51.23 °C          */
    int64_t  pressure;      /* Q24.8 Pa units  — e.g. 24674867 → 96386.2 Pa    */
    uint32_t humidity;      /* Q22.10 %rH units — divide by 1024 to get %rH    */
} bme280_data_t;

typedef struct bme280_ctrl_s
{
    spi_ctrl_t spi_ctrl;
    bme280_data_t data;
    bme280_calib_data_t cal_data;

} bme280_ctrl_t;

/*!
 * @brief Initialises the SPI peripheral and prepares the driver state.
 */
status_t bme280_init(bme280_ctrl_t *bme_ctl);

/*!
 * @brief Reads the factory calibration trim parameters from the sensor's NVM
 *        and stores them in bme_ctl->cal_data. Must be called once after init.
 */
status_t bme280_read_calibration(bme280_ctrl_t *bme_ctl);

/*!
 * @brief Performs a burst SPI read of the six output registers (0xF7–0xFE),
 *        reconstructs the three raw ADC values and calls the three compensation
 *        functions to update bme_ctl->data.
 */
void bme280_parse_data(bme280_ctrl_t *bme_ctl);

/*!
 * @brief Temperature compensation — int32 path (datasheet §4.2.3).
 *        Result in bme->data.temperature; units: 0.01 °C  (5123 → 51.23 °C).
 *        Also updates the global t_fine used by pressure and humidity.
 */
void bme280_temperature_int32(bme280_ctrl_t *bme, int32_t adc_val);

/*!
 * @brief Pressure compensation — int64 path (datasheet §4.2.3).
 *        Result in bme->data.pressure; units: Q24.8 Pa  (24674867 → 96386.2 Pa).
 *        bme280_temperature_int32() must have been called first.
 */
void bme280_pressure_int64(bme280_ctrl_t *bme, int32_t adc_val);

/*!
 * @brief Humidity compensation — int32 path (datasheet §4.2.3).
 *        Result in bme->data.humidity; units: Q22.10 %RH  (divide by 1024 for %RH).
 *        bme280_temperature_int32() must have been called first.
 */
void bme280_humidity_int32(bme280_ctrl_t *bme, int32_t adc_val);


#endif /* BME280_H_ */