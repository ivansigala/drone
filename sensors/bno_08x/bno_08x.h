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

typedef struct imu_ctrl_s
{
    spi_ctrl_t spi_ctrl;
    gpio_ctrl_t gpio_event;
    gpio_ctrl_t gpio_reset;

} imu_ctrl_t;

status_t bno_08x_init(imu_ctrl_t *imu, void* spi_callback, void* gpio_callback, sh2_SensorCallback_t sh2_callback);

#endif /* BNO_08X_H_ */