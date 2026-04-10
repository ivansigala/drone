/*
    mpu_9250.h
    Author: Diego
    Created on: 9, April 2026
 */

#ifndef MPU_9250_REG_H_
#define MPU_9250_REG_H_

/*
    mpu_9250_reg.h
    Author: Diego
    Created on: 9, April 2026
 */

// ==========================================
// MPU-9250 Register Map
// ==========================================

#define MPU9250_SELF_TEST_X_GYRO  0x00
#define MPU9250_SELF_TEST_Y_GYRO  0x01
#define MPU9250_SELF_TEST_Z_GYRO  0x02
#define MPU9250_SELF_TEST_X_ACCEL 0x0D
#define MPU9250_SELF_TEST_Y_ACCEL 0x0E
#define MPU9250_SELF_TEST_Z_ACCEL 0x0F

#define MPU9250_XG_OFFSET_H       0x13
#define MPU9250_XG_OFFSET_L       0x14
#define MPU9250_YG_OFFSET_H       0x15
#define MPU9250_YG_OFFSET_L       0x16
#define MPU9250_ZG_OFFSET_H       0x17
#define MPU9250_ZG_OFFSET_L       0x18

#define MPU9250_SMPLRT_DIV        0x19
#define MPU9250_CONFIG            0x1A
#define MPU9250_GYRO_CONFIG       0x1B
#define MPU9250_ACCEL_CONFIG      0x1C
#define MPU9250_ACCEL_CONFIG_2    0x1D
#define MPU9250_LP_ACCEL_ODR      0x1E
#define MPU9250_WOM_THR           0x1F

#define MPU9250_FIFO_EN           0x23
#define MPU9250_I2C_MST_CTRL      0x24
#define MPU9250_I2C_SLV0_ADDR     0x25
#define MPU9250_I2C_SLV0_REG      0x26
#define MPU9250_I2C_SLV0_CTRL     0x27

#define MPU9250_INT_PIN_CFG       0x37
#define MPU9250_INT_ENABLE        0x38
#define MPU9250_INT_STATUS        0x3A

// Accelerometer Data Registers
#define MPU9250_ACCEL_XOUT_H      0x3B
#define MPU9250_ACCEL_XOUT_L      0x3C
#define MPU9250_ACCEL_YOUT_H      0x3D
#define MPU9250_ACCEL_YOUT_L      0x3E
#define MPU9250_ACCEL_ZOUT_H      0x3F
#define MPU9250_ACCEL_ZOUT_L      0x40

// Temperature Data Registers
#define MPU9250_TEMP_OUT_H        0x41
#define MPU9250_TEMP_OUT_L        0x42

// Gyroscope Data Registers
#define MPU9250_GYRO_XOUT_H       0x43
#define MPU9250_GYRO_XOUT_L       0x44
#define MPU9250_GYRO_YOUT_H       0x45
#define MPU9250_GYRO_YOUT_L       0x46
#define MPU9250_GYRO_ZOUT_H       0x47
#define MPU9250_GYRO_ZOUT_L       0x48

// External Sensor Data (e.g. Magnetometer via I2C Aux)
#define MPU9250_EXT_SENS_DATA_00  0x49

#define MPU9250_USER_CTRL         0x6A
#define MPU9250_PWR_MGMT_1        0x6B
#define MPU9250_PWR_MGMT_2        0x6C
#define MPU9250_FIFO_COUNTH       0x72
#define MPU9250_FIFO_COUNTL       0x73
#define MPU9250_FIFO_R_W          0x74
#define MPU9250_WHO_AM_I          0x75 // Should return 0x71
#define MPU9250_XA_OFFSET_H       0x77
#define MPU9250_XA_OFFSET_L       0x78
#define MPU9250_YA_OFFSET_H       0x7A
#define MPU9250_YA_OFFSET_L       0x7B
#define MPU9250_ZA_OFFSET_H       0x7D
#define MPU9250_ZA_OFFSET_L       0x7E

// ==========================================
// AK8963 Magnetometer Registers (Accessible via I2C Master/Bypass)
// ==========================================
#define AK8963_I2C_ADDR           0x0C 
#define AK8963_WIA                0x00 // Device ID
#define AK8963_INFO               0x01
#define AK8963_ST1                0x02 // Status 1
#define AK8963_HXL                0x03 // X-axis data
#define AK8963_HXH                0x04
#define AK8963_HYL                0x05 // Y-axis data
#define AK8963_HYH                0x06
#define AK8963_HZL                0x07 // Z-axis data
#define AK8963_HZH                0x08
#define AK8963_ST2                0x09 // Status 2
#define AK8963_CNTL1              0x0A // Control 1
#define AK8963_CNTL2              0x0B // Control 2
#define AK8963_ASTC               0x0C // Self-Test
#define AK8963_ASAX               0x10 // X-axis sensitivity adjustment
#define AK8963_ASAY               0x11 // Y-axis sensitivity adjustment
#define AK8963_ASAZ               0x12 // Z-axis sensitivity adjustment

#endif /* MPU_9250_REG_H_ */