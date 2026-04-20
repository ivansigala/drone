/*
    bme280.c
    Author: Diego
    Created on: 13, April 2026
 */

#include "bme280.h"


/*************************************
 Global Variables
*************************************/
bme280_ctrl_t *g_barometer;
int32_t g_t_fine_32;

status_t bme280_init(bme280_ctrl_t *bme_ctl, void* callback)
{

    status_t spi_status;

#ifdef MCXN947
    spi_get_defaultconfig_bar(&(bme_ctl->spi_ctrl), callback);
#endif

    g_barometer = bme_ctl;
    g_t_fine_32 = 0;

    NVIC_SetPriority(EDMA_0_CH9_IRQn, 5);
    NVIC_SetPriority(EDMA_0_CH10_IRQn, 5);

    spi_status = spi_init(&(bme_ctl->spi_ctrl));
    if(spi_status != kStatus_Success){ 
        return spi_status;
    }

    return spi_status;
   
}


void bme280_parse_data(bme280_ctrl_t *bme_ctl)
{
    
}

/*!
 * @brief Calculates the temperature in °C with a resolution of 0.01 °C
 * @param adc_val adc value related to the temperature
 * @return output value of 5123 equals 51.23 °C
 */
void bme280_temperature_int32(bme280_ctrl_t *bme, int32_t adc_val)
{
    int32_t var1, var2;

    var1 = ( ( (adc_val>>3) - ( (int32_t)bme->cal_data.dig_T1<<1 ) ) * ( (int32_t)bme->cal_data.dig_T2) ) >> 11;
    var2 = ( ( ( ( (adc_val>>4) - ( (int32_t)bme->cal_data.dig_T1) ) * 
           ( ( adc_val >> 4) - ((int32_t)bme->cal_data.dig_T1)) ) >> 12 ) * ((int32_t)bme->cal_data.dig_T3) ) >> 14;

    g_t_fine_32 = var1 + var2;

    bme->data.temperature = (g_t_fine_32 * 5 + 128) >> 8;

}

/*!
 * @brief Calculates the pressure in Pa as unsigned integer in Q24.8 format (24 integer and 8 fractional bits)
 * @param adc_val adc value related to the temperature
 * @return output value of 24674867 represents 96386.2 Pa pressure/256
 */
void bme280_pressure_int64(bme280_ctrl_t *bme, int32_t adc_val)
{
    int64_t var1, var2, pressure;

    var1 = ((int64_t)g_t_fine_32) - 128000;
    var2 = var1 * var1 * (int64_t)bme->cal_data.dig_P6;
    var2 = var2 + ( ( var1 * (int64_t)bme->cal_data.dig_P5) << 17);
    var2 = var2 + ( ( (int64_t)bme->cal_data.dig_P4) << 35 );
    var1 = ( (var1 * var1 * (int64_t)bme->cal_data.dig_P3) >> 8 ) + ( (var1 * (int64_t)bme->cal_data.dig_P2) << 12);
    var1 = ( ( ((int64_t)1) << 47) + var1 ) * ((int64_t)bme->cal_data.dig_P1) >> 33;
    
    if(var1 == 0)
    {
        return;
    }

    pressure = 1048576 - adc_val;
    pressure = (((pressure << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)bme->cal_data.dig_P9) * (pressure<<13) * (pressure<<13) >> 25;
    var2 = ( ((int64_t)bme->cal_data.dig_P8) * pressure ) >> 19;
    bme->data.pressure = ((pressure + var1 + var2) >> 8) + (((int64_t)bme->cal_data.dig_P7) << 4);
      

}