/*
 * ina219_driver.c
 *
 * Original from the bare-metal frdmmcxn947_pwm_cm33_core0 project (Feb 2026).
 */

#include "ina219_driver.h"

/* INA219 register addresses */
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNTVOLTAGE 0x01
#define INA219_REG_BUSVOLTAGE   0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

/* Pre-computed register values for the 32V / 2A range with a 0.1Ω shunt. */
#define INA219_CONFIG_32V_2A    0x399F  /* 32V, 320mV shunt range, 12-bit ADC, continuous */
#define INA219_CAL_VALUE_32V_2A 4096    /* Yields 100µA per current LSB */


/* --- Internal helpers --- */
static status_t INA219_WriteReg16(LPI2C_Type *base, uint8_t i2cAddr, uint8_t reg, uint16_t value)
{
    lpi2c_master_transfer_t masterXfer = {0};

    /* INA219 is big-endian on the wire. */
    uint8_t data[2] = {
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)(value & 0xFF)
    };

    masterXfer.slaveAddress = i2cAddr;
    masterXfer.direction = kLPI2C_Write;
    masterXfer.subaddress = reg;
    masterXfer.subaddressSize = 1;
    masterXfer.data = data;
    masterXfer.dataSize = 2;
    masterXfer.flags = kLPI2C_TransferDefaultFlag;

    return LPI2C_MasterTransferBlocking(base, &masterXfer);
}

static status_t INA219_ReadReg16(LPI2C_Type *base, uint8_t i2cAddr, uint8_t reg, int16_t *value)
{
    lpi2c_master_transfer_t masterXfer = {0};
    uint8_t data[2] = {0};
    status_t status;

    masterXfer.slaveAddress = i2cAddr;
    masterXfer.direction = kLPI2C_Read;
    masterXfer.subaddress = reg;
    masterXfer.subaddressSize = 1;
    masterXfer.data = data;
    masterXfer.dataSize = 2;
    masterXfer.flags = kLPI2C_TransferDefaultFlag;

    status = LPI2C_MasterTransferBlocking(base, &masterXfer);

    if (status == kStatus_Success) {
        *value = (int16_t)((data[0] << 8) | data[1]);
    }

    return status;
}

/* --- Public API --- */

status_t INA219_Init(ina219_handle_t *handle, LPI2C_Type *base, uint8_t address)
{
    status_t status;

    handle->i2cBase = base;
    handle->i2cAddress = address;

    /* Soft reset (Config Register MSB = 1). The chip ACKs the write, then
     * snaps the Config register back to its power-on-reset value. */
    status = INA219_WriteReg16(handle->i2cBase, handle->i2cAddress, INA219_REG_CONFIG, 0x8000);

    /* Datasheet asks for a short delay after reset before more access. */
    SDK_DelayAtLeastUs(100000, CLOCK_GetFreq(kCLOCK_CoreSysClk));

    /* Read the Config register back and verify it matches the POR default.
     * This single round-trip catches the common bring-up problems:
     *   - wrong slave address (no ACK -> kStatus_Fail above already)
     *   - bus held low / no pull-ups (transfer timeout)
     *   - reads succeeding but returning garbage (a different chip on the
     *     address, or signal integrity issues) -> mismatch handled here. */
    
    status = INA219_CheckPresence(handle);
    
    return status;

}

status_t INA219_CheckPresence(ina219_handle_t *handle)
{
    int16_t  reg = 0;
    status_t status = INA219_ReadReg16(handle->i2cBase,
                                        handle->i2cAddress,
                                        INA219_REG_CONFIG,
                                        &reg);
    if (status != kStatus_Success) {
        return status;
    }
    if ((uint16_t)reg != INA219_CONFIG_POR_VALUE) {
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t INA219_Calibrate_32V_2A(ina219_handle_t *handle)
{
    status_t status;

    /* For Rshunt = 0.1Ω, VBUS_max = 32V, I_max = 2A:
     *   Current LSB = 100µA → divide raw by 10 to get mA.
     *   Power   LSB = 2mW   → multiply raw by 2 to get mW. */
    handle->calibrationValue   = INA219_CAL_VALUE_32V_2A;
    handle->currentDivider_mA  = 10.0f;
    handle->powerMultiplier_mW = 2.0f;

    status = INA219_WriteReg16(handle->i2cBase, handle->i2cAddress, INA219_REG_CALIBRATION, handle->calibrationValue);
    if (status != kStatus_Success) return status;

    status = INA219_WriteReg16(handle->i2cBase, handle->i2cAddress, INA219_REG_CONFIG, INA219_CONFIG_32V_2A);
    return status;
}

status_t INA219_ReadSensor(ina219_handle_t *handle)
{
    status_t status;

    /* Re-write calibration in case the chip was reset by a glitch. */
    // status = INA219_WriteReg16(handle->i2cBase, handle->i2cAddress, INA219_REG_CALIBRATION, handle->calibrationValue);
    // if(status != kStatus_Success) return status;

    // status = INA219_ReadReg16(handle->i2cBase, handle->i2cAddress, INA219_REG_SHUNTVOLTAGE, &handle->shuntVoltageRaw);
    // if(status != kStatus_Success) return status;

    // status = INA219_ReadReg16(handle->i2cBase, handle->i2cAddress, INA219_REG_BUSVOLTAGE, &handle->busVoltageRaw);
    // if(status != kStatus_Success) return status;

    status = INA219_ReadReg16(handle->i2cBase, handle->i2cAddress, INA219_REG_CURRENT, &handle->currentRaw);
    if(status != kStatus_Success) return status;

    /* Decode raw -> physical units */

    /* Shunt voltage: 10 µV per LSB → 0.01 mV per LSB. */
    // handle->shuntVoltage_mV = (float)handle->shuntVoltageRaw * 0.01f;

    // /* Bus voltage: register is left-aligned by 3, then 4 mV per LSB. */
    // handle->busVoltage_V = (float)((handle->busVoltageRaw >> 3) * 4) * 0.001f;

    /* Current: 100 µA per LSB at this calibration. */
    handle->current_mA = (float)handle->currentRaw / handle->currentDivider_mA;

    return kStatus_Success;
}
