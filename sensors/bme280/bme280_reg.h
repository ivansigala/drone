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


/* Calibration registers — burst-read helpers */
/* Block 1: dig_T1..dig_T3, dig_P1..dig_P9  (24 bytes, all little-endian) */
#define BME280_CALIB_T_P_START   0x88
#define BME280_CALIB_T_P_LEN     24U

/* dig_H1 sits alone at 0xA1 (1 byte, unsigned) */
#define BME280_CALIB_H1_REG      0xA1

/* Block 2: dig_H2..dig_H6  (7 bytes) */
#define BME280_CALIB_H_START     0xE1
#define BME280_CALIB_H_LEN       7U

/* Burst read of all six output data registers: press_msb..hum_lsb (8 bytes) */
#define BME280_DATA_BURST_START  0xF7
#define BME280_DATA_BURST_LEN    8U

/* SPI read/write bit (bit 7 of the address byte) */
#define BME280_SPI_RD_MASK       0x80U
#define BME280_SPI_WR_MASK       0x7FU

/* Chip ID — read from ID_REG (0xD0); used to verify SPI communication */
#define BME280_CHIP_ID           0x60U

/* Soft-reset command written to RESET_REG (0xE0) */
#define BME280_SOFT_RESET_CMD    0xB6U

/* Operating modes — bits [1:0] of CTRL_MEAS_REG */
#define BME280_MODE_SLEEP        0x00U
#define BME280_MODE_FORCED       0x01U
#define BME280_MODE_NORMAL       0x03U

/* Oversampling settings for osrs_t, osrs_p, osrs_h */
#define BME280_OSRS_SKIP         0x00U   /* measurement skipped — output is sentinel */
#define BME280_OSRS_X1           0x01U
#define BME280_OSRS_X2           0x02U
#define BME280_OSRS_X4           0x03U
#define BME280_OSRS_X8           0x04U
#define BME280_OSRS_X16          0x05U

/* Default register values used by bme280_init():
 *   CTRL_HUM  must be written BEFORE CTRL_MEAS for the setting to take effect.
 *   CTRL_MEAS: osrs_t[7:5] | osrs_p[4:2] | mode[1:0]
 *   CONFIG   : t_sb[7:5]   | filter[4:2] | spi3w_en[0]
 *              t_sb=000 (0.5 ms standby), filter=000 (off)                      */
#define BME280_CTRL_HUM_VAL      ( BME280_OSRS_X1 )                        /* 0x01 */
#define BME280_CTRL_MEAS_VAL     ( (BME280_OSRS_X1 << 5) | \
                                   (BME280_OSRS_X1 << 2) | \
                                    BME280_MODE_NORMAL )                    /* 0x27 */
#define BME280_CONFIG_VAL        0x00U




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