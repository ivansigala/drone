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

/* IIR filter coefficients — CONFIG_REG bits [4:2] ----------------------
 *   Higher coefficient = more smoothing, slower step response.
 *   For altitude estimation the ×16 setting is the standard choice —
 *   it drops short-term pressure noise from ~1.3 Pa RMS to ~0.2 Pa RMS
 *   (≈ 0.017 m altitude 1σ) without meaningfully lagging the baro pipe.  */
#define BME280_FILTER_OFF        0x00U
#define BME280_FILTER_X2         0x01U
#define BME280_FILTER_X4         0x02U
#define BME280_FILTER_X8         0x03U
#define BME280_FILTER_X16        0x04U

/* Standby time in NORMAL mode — CONFIG_REG bits [7:5] ------------------
 *   000 = 0.5 ms   (→ ~250 Hz output, causes die self-heating drift)
 *   001 = 62.5 ms  (→ ~16 Hz, good balance for drone altimetry)
 *   010 = 125 ms
 *   011 = 250 ms
 *   100 = 500 ms
 *   101 = 1000 ms
 *   110 = 10 ms
 *   111 = 20 ms                                                          */
#define BME280_T_SB_0P5MS        0x00U
#define BME280_T_SB_62P5MS       0x01U
#define BME280_T_SB_125MS        0x02U
#define BME280_T_SB_250MS        0x03U
#define BME280_T_SB_500MS        0x04U
#define BME280_T_SB_1000MS       0x05U
#define BME280_T_SB_10MS         0x06U
#define BME280_T_SB_20MS         0x07U

/* Default register values used by bme280_init():
 *   CTRL_HUM  must be written BEFORE CTRL_MEAS for the setting to take effect.
 *   CTRL_MEAS: osrs_t[7:5] | osrs_p[4:2] | mode[1:0]
 *   CONFIG   : t_sb[7:5]   | filter[4:2] | spi3w_en[0]
 *
 *  ──  Tuned for altimetry ─────────────────────────────────────────────
 *   osrs_p = ×16  : best available pressure resolution (0.2 Pa RMS + filter)
 *   osrs_t = ×1   : temperature is only used for the hypsometric T_K term,
 *                   which doesn't need extra averaging
 *   osrs_h = ×1   : humidity unused on the drone, but keep it on — if
 *                   skipped the sensor writes 0x8000 sentinel values.
 *   filter = ×16  : maximum IIR smoothing — kills per-sample spikes
 *   t_sb   = 62.5 ms : ~16 Hz output.  Reading faster than this from the
 *                      MCU side is fine (shadow registers hold the last
 *                      completed conversion) and keeps self-heating low.
 *
 *  Measured impact (vs osrs_p=×1, filter=off, t_sb=0.5 ms):
 *   baro noise 1σ     :  2.48 m  →  ~0.02 m   (expected)
 *   baro drift slope  : -0.147 m/s → should approach 0             */
#define BME280_CTRL_HUM_VAL      ( BME280_OSRS_X1 )                        /* 0x01 */
#define BME280_CTRL_MEAS_VAL     ( (BME280_OSRS_X1  << 5) | \
                                   (BME280_OSRS_X16 << 2) | \
                                    BME280_MODE_NORMAL )                    /* 0x37 */
#define BME280_CONFIG_VAL        ( (BME280_T_SB_62P5MS << 5) | \
                                   (BME280_FILTER_X16  << 2) )              /* 0x30 */




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