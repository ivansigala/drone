/*
    bmp581_reg.h
    Author: Diego
    Created on: 30, April 2026

    Register map and bit-field constants for the Bosch BMP581 barometric
    pressure / temperature sensor.

    Datasheet reference: BMP581 datasheet rev 1.7 (2023-03), §5 Register map.

    Notes vs BME280:
      - All registers live in a different address range (0x00..0x7F).
      - Pressure & temperature data are *already compensated* on-chip and
        come out in fixed-point engineering units; no NVM trim block to
        read back.  See bmp581_parse_data() in bmp581.c.
      - SPI framing: although the BMP581 datasheet (§7.1.2) describes a
        one-byte dummy phase between the address echo and the first data
        byte, this LPSPI driver / SPI mode combo does NOT see that dummy
        on the wire.  Verified empirically with a logic-analyzer capture
        of a CHIP_ID read: data appears at rx[1] immediately after the
        address phase, identical to the BME280 read framing.  The driver
        therefore treats READ as { address, N data } — no skip byte.
*/


#ifndef BMP581_REG_H_
#define BMP581_REG_H_


/* ---- Identification & status ---------------------------------------- */
#define BMP581_CHIP_ID_REG          0x01    /* expected value 0x50            */
#define BMP581_REV_ID_REG           0x02
#define BMP581_CHIP_STATUS_REG      0x11

/* ---- Interface / interrupt control ---------------------------------- */
#define BMP581_DRIVE_CONFIG_REG     0x13
#define BMP581_INT_CONFIG_REG       0x14
#define BMP581_INT_SOURCE_REG       0x15
#define BMP581_FIFO_CONFIG_REG      0x16
#define BMP581_FIFO_COUNT_REG       0x17
#define BMP581_FIFO_SEL_REG         0x18

/* ---- Output data registers (24-bit values, little-endian) ----------- */
/* Temperature: signed 24-bit, Q8.16 °C   →  raw / 2^16  = °C            */
#define BMP581_TEMP_DATA_XLSB_REG   0x1D
#define BMP581_TEMP_DATA_LSB_REG    0x1E
#define BMP581_TEMP_DATA_MSB_REG    0x1F
/* Pressure:    unsigned 24-bit, Q18.6 Pa →  raw / 2^6   = Pa            */
#define BMP581_PRESS_DATA_XLSB_REG  0x20
#define BMP581_PRESS_DATA_LSB_REG   0x21
#define BMP581_PRESS_DATA_MSB_REG   0x22

/* ---- Status / FIFO data --------------------------------------------- */
#define BMP581_INT_STATUS_REG       0x27
#define BMP581_STATUS_REG           0x28
#define BMP581_FIFO_DATA_REG        0x29

/* ---- DSP / OSR / ODR configuration ---------------------------------- */
#define BMP581_DSP_CONFIG_REG       0x30
#define BMP581_DSP_IIR_REG          0x31
#define BMP581_OOR_THR_P_LSB_REG    0x32
#define BMP581_OOR_THR_P_MSB_REG    0x33
#define BMP581_OOR_RANGE_REG        0x34
#define BMP581_OOR_CONFIG_REG       0x35
#define BMP581_OSR_CONFIG_REG       0x36
#define BMP581_ODR_CONFIG_REG       0x37
#define BMP581_OSR_EFF_REG          0x38

/* ---- Command register ----------------------------------------------- */
#define BMP581_CMD_REG              0x7E

/* ---- Burst-read helpers --------------------------------------------- */
/* Six output bytes from 0x1D..0x22 — temp_xlsb..press_msb               */
#define BMP581_DATA_BURST_START     BMP581_TEMP_DATA_XLSB_REG
#define BMP581_DATA_BURST_LEN       6U

/* ---- SPI framing ----------------------------------------------------
 * Same convention as the BME280 driver: bit 7 of the address byte
 * selects READ (1) or WRITE (0).  Total READ length is
 *     1 (address) + N (data)   bytes
 * with meaningful data at rx[1..N].  WRITE transfers are
 * { address, data } — two bytes total.
 *
 * BMP581_SPI_READ_DUMMY_LEN is kept (set to 0) so any future caller that
 * does     1U + BMP581_SPI_READ_DUMMY_LEN + N     compiles unchanged if
 * a different SPI mode ever does need a dummy byte inserted.            */
#define BMP581_SPI_RD_MASK          0x80U
#define BMP581_SPI_WR_MASK          0x7FU
#define BMP581_SPI_READ_DUMMY_LEN   0U

/* ---- Identification values ------------------------------------------ */
#define BMP581_CHIP_ID              0x50U   /* CHIP_ID @0x01 for BMP581       */

/* ---- Soft-reset ------------------------------------------------------ */
/* Write 0xB6 to CMD (0x7E); device needs ~2 ms to be ready (datasheet §5.7) */
#define BMP581_SOFT_RESET_CMD       0xB6U

/* ---- INT_STATUS bits (0x27) ----------------------------------------- */
#define BMP581_INT_STATUS_DRDY      (1U << 0)   /* new data available        */
#define BMP581_INT_STATUS_FIFO_FULL (1U << 1)
#define BMP581_INT_STATUS_FIFO_THS  (1U << 2)
#define BMP581_INT_STATUS_OOR_P     (1U << 3)
#define BMP581_INT_STATUS_POR       (1U << 4)   /* set after power-on/reset  */

/* ---- INT_CONFIG (0x14) bit fields -----------------------------------
 *   bit 0  int_mode   0 = pulsed, 1 = latched
 *   bit 1  int_pol    0 = active-low, 1 = active-high
 *   bit 2  int_od     0 = push-pull, 1 = open-drain
 *   bit 3  int_en     1 = INT pin enabled                                  */
#define BMP581_INT_MODE_PULSED      (0U << 0)
#define BMP581_INT_MODE_LATCHED     (1U << 0)
#define BMP581_INT_POL_ACTIVE_LOW   (0U << 1)
#define BMP581_INT_POL_ACTIVE_HIGH  (1U << 1)
#define BMP581_INT_OD_PUSH_PULL     (0U << 2)
#define BMP581_INT_OD_OPEN_DRAIN    (1U << 2)
#define BMP581_INT_EN_BIT           (1U << 3)

/* ---- INT_SOURCE (0x15) interrupt-source enables --------------------- */
#define BMP581_INT_SRC_DRDY_DATA    (1U << 0)
#define BMP581_INT_SRC_FIFO_FULL    (1U << 1)
#define BMP581_INT_SRC_FIFO_THS     (1U << 2)
#define BMP581_INT_SRC_OOR_P        (1U << 3)

/* Default INT config: active-low, push-pull, pulsed, INT pin enabled.
 * Matches the BNO085 falling-edge convention used elsewhere in the
 * project so a single GPIO-edge ISR pattern works for both sensors.
 *
 *   pulsed   : the INT pin self-clears after a short pulse — the MCU
 *              does not need to read INT_STATUS to deassert.  Reading
 *              INT_STATUS is still useful if you want to know *which*
 *              source asserted, but is not required to keep events flowing.
 *   active-low push-pull : matches gpio_attach_interrupt() falling-edge
 *              setup used for the BNO085 HINT line.                       */
#define BMP581_INT_CONFIG_VAL       ( BMP581_INT_EN_BIT          | \
                                      BMP581_INT_OD_PUSH_PULL    | \
                                      BMP581_INT_POL_ACTIVE_LOW  | \
                                      BMP581_INT_MODE_PULSED )       /* 0x08 */

/* Only DRDY drives the line — FIFO/OOR sources stay disabled.            */
#define BMP581_INT_SOURCE_VAL       ( BMP581_INT_SRC_DRDY_DATA )       /* 0x01 */

/* ---- STATUS bits (0x28) --------------------------------------------- */
#define BMP581_STATUS_CORE_RDY      (1U << 0)
#define BMP581_STATUS_NVM_RDY       (1U << 1)
#define BMP581_STATUS_NVM_ERR       (1U << 2)
#define BMP581_STATUS_NVM_CMD_ERR   (1U << 3)

/* ---- ODR_CONFIG (0x37) --------------------------------------------- */
/*  bit 7      DEEP_DIS   (1 = deep standby disabled — required for normal mode)
 *  bits [6:2] ODR        (output data rate, 5-bit field, see table below)
 *  bits [1:0] PWR_MODE                                                    */
#define BMP581_PWR_MODE_STANDBY     0x00U
#define BMP581_PWR_MODE_NORMAL      0x01U
#define BMP581_PWR_MODE_FORCED      0x02U
#define BMP581_PWR_MODE_NON_STOP    0x03U   /* a.k.a. continuous           */

#define BMP581_ODR_240_HZ           0x00U
#define BMP581_ODR_218P537_HZ       0x01U
#define BMP581_ODR_199P111_HZ       0x02U
#define BMP581_ODR_179P2_HZ         0x03U
#define BMP581_ODR_160_HZ           0x04U
#define BMP581_ODR_149P333_HZ       0x05U
#define BMP581_ODR_129P524_HZ       0x06U
#define BMP581_ODR_110P155_HZ       0x07U
#define BMP581_ODR_99P555_HZ        0x08U   /* ≈100 Hz                     */
#define BMP581_ODR_89P6_HZ          0x09U
#define BMP581_ODR_79P556_HZ        0x0AU
#define BMP581_ODR_69P818_HZ        0x0BU
#define BMP581_ODR_59P733_HZ        0x0CU
#define BMP581_ODR_49P778_HZ        0x0DU   /* ≈50 Hz                      */
#define BMP581_ODR_39P911_HZ        0x0EU
#define BMP581_ODR_30P149_HZ        0x0FU
#define BMP581_ODR_25_HZ            0x10U
#define BMP581_ODR_20_HZ            0x11U
#define BMP581_ODR_15_HZ            0x12U
#define BMP581_ODR_10_HZ            0x13U
#define BMP581_ODR_5_HZ             0x14U
#define BMP581_ODR_4_HZ             0x15U
#define BMP581_ODR_3_HZ             0x16U
#define BMP581_ODR_2_HZ             0x17U
#define BMP581_ODR_1_HZ             0x18U

#define BMP581_DEEP_DIS_BIT         (1U << 7)

/* ---- OSR_CONFIG (0x36) --------------------------------------------- */
/*  bit 6      PRESS_EN  (1 = pressure measurement enabled)
 *  bits [5:3] OSR_P     (pressure  oversampling)
 *  bits [2:0] OSR_T     (temperature oversampling)                       */
#define BMP581_OSR_X1               0x00U
#define BMP581_OSR_X2               0x01U
#define BMP581_OSR_X4               0x02U
#define BMP581_OSR_X8               0x03U
#define BMP581_OSR_X16              0x04U
#define BMP581_OSR_X32              0x05U
#define BMP581_OSR_X64              0x06U
#define BMP581_OSR_X128             0x07U

#define BMP581_PRESS_EN_BIT         (1U << 6)

/* ---- DSP_CONFIG (0x30) --------------------------------------------- */
/*  bit 0  IIR_FLUSH_FORCED_EN
 *  bit 1  SHDW_SEL_IIR_T   (1 = shadow data regs return IIR-filtered T)
 *  bit 2  FIFO_SEL_IIR_T
 *  bit 3  SHDW_SEL_IIR_P   (1 = shadow data regs return IIR-filtered P)
 *  bit 4  FIFO_SEL_IIR_P
 *  bits [7:5] OOR_SEL_IIR_P                                              */
#define BMP581_DSP_IIR_FLUSH_FORCED (1U << 0)
#define BMP581_DSP_SHDW_SEL_IIR_T   (1U << 1)
#define BMP581_DSP_FIFO_SEL_IIR_T   (1U << 2)
#define BMP581_DSP_SHDW_SEL_IIR_P   (1U << 3)
#define BMP581_DSP_FIFO_SEL_IIR_P   (1U << 4)

/* ---- DSP_IIR (0x31) IIR coefficients ------------------------------- */
/*  bits [5:3] SET_IIR_P   bits [2:0] SET_IIR_T                            */
#define BMP581_IIR_BYPASS           0x00U
#define BMP581_IIR_COEF_1           0x01U
#define BMP581_IIR_COEF_3           0x02U
#define BMP581_IIR_COEF_7           0x03U
#define BMP581_IIR_COEF_15          0x04U
#define BMP581_IIR_COEF_31          0x05U
#define BMP581_IIR_COEF_63          0x06U
#define BMP581_IIR_COEF_127         0x07U


/* ─────────────────────────────────────────────────────────────────────
 *  Default register values used by bmp581_init():
 *
 *  Tuned for drone altimetry on the same SPI bus the BME280 used.
 *  The BMP581 has substantially better intrinsic pressure resolution
 *  than the BME280 (≈0.6 Pa RMS at OSR_P×8 vs ≈1.3 Pa RMS at OSR_P×16
 *  for the BME280), so we can run at a higher ODR with comparable noise.
 *
 *  OSR_P = ×8   : ~0.6 Pa RMS, 17 ms conversion → fits inside 50 Hz ODR
 *  OSR_T = ×1   : temperature is only used for the hypsometric T_K term
 *  ODR   = 50 Hz: ~3× the effective rate of the BME280 config (62.5 ms)
 *  IIR_P = 127  : strongest available IIR — cuts spike noise to <0.1 Pa
 *  IIR_T = bypass : do not lag temperature, it changes slowly anyway
 *  Shadow data registers are routed through the IIR filter for pressure
 *  so a plain burst read of 0x1D..0x22 returns the filtered value.
 *  ──────────────────────────────────────────────────────────────────── */
#define BMP581_OSR_CONFIG_VAL       ( BMP581_PRESS_EN_BIT          | \
                                      (BMP581_OSR_X8  << 3)        | \
                                       BMP581_OSR_X1 )                 /* 0x58 */

#define BMP581_ODR_CONFIG_VAL       ( BMP581_DEEP_DIS_BIT           | \
                                      (BMP581_ODR_49P778_HZ << 2)   | \
                                       BMP581_PWR_MODE_NORMAL )         /* 0xB5 */

#define BMP581_DSP_CONFIG_VAL       ( BMP581_DSP_SHDW_SEL_IIR_P )       /* 0x08 */

#define BMP581_DSP_IIR_VAL          ( (BMP581_IIR_COEF_127 << 3)    | \
                                       BMP581_IIR_BYPASS )              /* 0x38 */


#endif /* BMP581_REG_H_ */
