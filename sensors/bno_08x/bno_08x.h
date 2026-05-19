/*
    bno_08x.h
    Author: Diego
    Created on: 10, April 2026
 */

#ifndef BNO_08X_H_
#define BNO_08X_H_

#ifdef MCXN947
#include "spi_driver_mcxn947.h"
#include "gpio_driver_mcxn947.h"
#include "fsl_debug_console.h"
#endif

#include "sh2.h"
#include "sh2_err.h"
#include "sh2_hal.h"
#include "sh2_SensorValue.h"
#include "sh2_util.h"
#include "FreeRTOS.h"
#include "task.h"

#define SH2_HAL_MAX_TRANSFER_IN_APP 512 // Max packet size for BNO085 is 512 bytes
#define SH2_HAL_MAX_TRANSFER_OUT_APP 128 // Max packet size for BNO085 is 128 bytes

typedef struct imu_ctrl_s
{
    spi_ctrl_t spi_ctrl;
    gpio_ctrl_t gpio_event;
    gpio_ctrl_t gpio_reset;
    gpio_ctrl_t gpio_ps0;

} imu_ctrl_t;

status_t bno_08x_init(imu_ctrl_t *imu, void* gpio_callback);
status_t bno_08x_start(sh2_SensorCallback_t sh2_callback);
status_t bno_08x_configure_sensors(void);
bool bno_08x_reset_occurred(void);
status_t bno_08x_tare(void);

#endif /* BNO_08X_H_ */