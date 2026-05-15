/*
    vl53l0x_reg.h
    Author: Diego
    Created on: 5, May 2026

    Register and constant definitions for the ST VL53L0X Time-of-Flight
    ranging sensor.

    Datasheet reference: VL53L0X DS11555 Rev 6 (June 2024).

    A note on register coverage:
      ST does NOT publish a complete register map in the public datasheet —
      §4.2 "I²C interface - reference registers" only lists five reference
      registers used to validate the bus.  All other addresses below come
      from ST's open-source VL53L0X API (Bosch-style header constants
      reverse-engineered/published by the Pololu and Adafruit libraries).
      They are stable across silicon revisions and are what every
      open-source driver for this part uses.
*/

#ifndef VL53L0X_REG_H_
#define VL53L0X_REG_H_


/* ---- I²C bus ---------------------------------------------------------- */
/* Datasheet §2.1 quotes "Address 0x52" — that is the *8-bit* form that
 * includes the R/W bit (0x52 = write, 0x53 = read).  The 7-bit address
 * the LPI2C driver wants is 0x52 >> 1 = 0x29.                            */
#define VL53L0X_I2C_ADDR_DEFAULT        0x29U


/* ---- Identification (datasheet §4.2) ---------------------------------- */
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xC0U  /* expect 0xEE */
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xC2U  /* expect 0x10 */
#define VL53L0X_MODEL_ID_EXPECTED                   0xEEU


/* ---- System / interrupt ----------------------------------------------- */
#define VL53L0X_REG_SYSRANGE_START                  0x00U
#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG          0x01U
#define VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD  0x04U
#define VL53L0X_REG_SYSTEM_RANGE_CONFIG             0x09U
#define VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_CONFIG    0x0AU
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR          0x0BU
#define VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH         0x84U

/* SYSRANGE_START values                                                   */
#define VL53L0X_SYSRANGE_MODE_SINGLESHOT            0x01U
#define VL53L0X_SYSRANGE_MODE_BACKTOBACK            0x02U  /* continuous */
#define VL53L0X_SYSRANGE_MODE_TIMED                 0x04U
#define VL53L0X_SYSRANGE_MODE_HISTOGRAM             0x08U

/* SYSTEM_INTERRUPT_GPIO_CONFIG values                                     */
#define VL53L0X_GPIO_FUNC_DISABLED                  0x00U
#define VL53L0X_GPIO_FUNC_LEVEL_LOW                 0x01U
#define VL53L0X_GPIO_FUNC_LEVEL_HIGH                0x02U
#define VL53L0X_GPIO_FUNC_OUT_OF_WINDOW             0x03U
#define VL53L0X_GPIO_FUNC_NEW_SAMPLE_READY          0x04U


/* ---- Result registers ------------------------------------------------- */
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13U
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14U
/* Bytes 10..11 of the RESULT_RANGE_STATUS block hold the 16-bit big-endian
 * range in millimetres — the canonical "give me the distance" address.    */
#define VL53L0X_REG_RESULT_RANGE_MM                 (0x14U + 10U)


/* ---- Tuning / calibration registers used by the minimal init ---------- */
#define VL53L0X_REG_MSRC_CONFIG_CONTROL             0x60U
#define VL53L0X_REG_FINAL_RANGE_CONFIG_RATE_LIMIT_LO 0x44U  /* 16-bit BE */


/* SYSTEM_SEQUENCE_CONFIG: enables every step the sensor's state machine
 * runs per measurement.  0xFF = TCC | DSS | MSRC | PRE_RANGE | FINAL_RANGE,
 * which is what the ST API leaves it at for a normal measurement.        */
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG_DEFAULT      0xE8U


#endif /* VL53L0X_REG_H_ */
