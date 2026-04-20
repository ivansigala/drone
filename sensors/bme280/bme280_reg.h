/*
    bme280_reg.h
    Author: Diego
    Created on: 13, April 2026
 */

#ifndef BME280_REG_H_
#define BME280_REG_H_


/* General important registers */
#define HUM_LSB_REG 0xFE
#define HUM_MSB_REG 0xFD
#define TEMP_XLSB_REG 0xFC
#define TEMP_LSB_REG 0xFB
#define TEMP_MSB_REG 0xFA
#define RESS_XLSB_REG 0xF9
#define PRESS_LSB_REG 0xF8
#define PRESS_MSB_REG 0xF7
#define CONFIG_REG 0xF5
#define CTRL_MEAS_REG 0xF4
#define STATUS_REG 0xF3
#define CTRL_HUM_REG 0xF2
#define RESET_REG 0xE0
#define ID_REG 0xD0


/* Calibration registers */




/* Compensation parameter storage */
#define DIG_T1_LSB_REG 0x88
#define DIG_T1_MSB_REG 0x89
#define DIG_T2_LSB_REG 0x8A
#define DIG_T2_MSB_REG 0x8B
#define DIG_T3_LSB_REG 0x8C
#define DIG_T3_MSB_REG 0x8D
#define DIG_P1_LSB_REG 0x8E
#define DIG_P1_MSB_REG 0x8F
#define DIG_P2_LSB_REG 0x90
#define DIG_P2_MSB_REG 0x91
#define DIG_P3_LSB_REG 0x92
#define DIG_P3_MSB_REG 0x93
#define DIG_P4_LSB_REG 0x94
#define DIG_P4_MSB_REG 0x95
#define DIG_P5_LSB_REG 0x96
#define DIG_P5_MSB_REG 0x97
#define DIG_P6_LSB_REG 0x98
#define DIG_P6_MSB_REG 0x99
#define DIG_P7_LSB_REG 0x9A
#define DIG_P7_MSB_REG 0x9B
#define DIG_P8_LSB_REG 0x9C
#define DIG_P8_MSB_REG 0x9D
#define DIG_P9_LSB_REG 0x9E
#define DIG_P9_MSB_REG 0x9F
#define DIG_H1_REG 0xA1
#define DIG_H2_LSB_REG 0xE1
#define DIG_H2_MSB_REG 0xE2
#define DIG_H3_REG 0xE3
#define DIG_H4_MSB_REG 0xE4
#define DIG_H4_LSB_REG 0xE5
#define DIG_H5_MSB_REG 0xE6
#define DIG_H6 0xE7


#endif /* BME280_REG_H_ */