/*
 * ina219_driver.h
 *
 * Original from the bare-metal frdmmcxn947_pwm_cm33_core0 project (Feb 2026).
 * Brought into the FreeRTOS PID project to read motor current via I2C.
 */

#ifndef INA219_DRIVER_H_
#define INA219_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include "fsl_common.h"
#include "fsl_lpi2c.h"

/* Default 7-bit I2C address (A0/A1 grounded). */
#define INA219_ADDR_DEFAULT     0x40

/* The INA219 has no true WHO_AM_I register, but its Configuration register
 * (0x00) has a fixed power-on-reset value. Reading it back right after a
 * soft reset is the standard "is the chip alive?" check. */
#define INA219_CONFIG_POR_VALUE 0x399F

/* Driver handle: holds the I2C peripheral pointer plus calibration and
 * the most recent measurements. The blocking transfers in this driver are
 * NOT FreeRTOS-aware — call them only from task context, never from an
 * ISR. */
typedef struct {
    /* Hardware */
    LPI2C_Type *i2cBase;
    uint8_t     i2cAddress;

    /* Calibration */
    uint32_t calibrationValue;
    float    currentDivider_mA;
    float    powerMultiplier_mW;

    /* Last decoded values */
    float busVoltage_V;
    float shuntVoltage_mV;
    float current_mA;
    float power_mW;

    /* Last raw register reads */
    int16_t currentRaw;
    int16_t busVoltageRaw;
    int16_t shuntVoltageRaw;
    int16_t powerRaw;
} ina219_handle_t;

/* Reset the chip and stash I2C base + slave address. After the reset the
 * Config register is read back and compared against INA219_CONFIG_POR_VALUE;
 * a mismatch returns kStatus_Fail so a wiring or addressing problem is
 * caught at boot instead of silently giving you 0 mA forever. */
status_t INA219_Init(ina219_handle_t *handle, LPI2C_Type *base, uint8_t address);

/* Lightweight presence check: read Config register and compare against
 * INA219_CONFIG_POR_VALUE. The chip's Config register only matches POR
 * immediately after reset or before INA219_Calibrate_*() is called, so use
 * this after Init() / Reset() but before applying calibration, or call
 * INA219_Init() again first. */
status_t INA219_CheckPresence(ina219_handle_t *handle);

/* Configure for 32V bus / 2A max range (0.1Ω shunt). 100µA per current LSB. */
status_t INA219_Calibrate_32V_2A(ina219_handle_t *handle);

/* Read all telemetry registers and decode into the handle. */
status_t INA219_ReadSensor(ina219_handle_t *handle);

#endif /* INA219_DRIVER_H_ */
