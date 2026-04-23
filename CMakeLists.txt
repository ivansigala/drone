
# Copyright 2026 NXP
#
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.22.0)

cmake_path(APPEND conf_file_path ${CMAKE_CURRENT_LIST_DIR} ${board}_${core_id} prj.conf)
set(CONF_FILE ${conf_file_path} ${CONF_FILE})
include(${SdkRootDirPath}/cmake/extension/mcux.cmake)

project(freertos_hello LANGUAGES C CXX ASM PROJECT_BOARD_PORT_PATH ${board_root}/${board}/freertos_examples/freertos_hello)


include("${board}_${core_id}/board_files.cmake")
include(${SdkRootDirPath}/CMakeLists.txt)


mcux_add_source(
    SOURCES main.c
            kalman_z/kalman_z.c
            sensors/mpu_9250/mpu_9250.c
            sensors/bno_08x/bno_08x.c
            sensors/bme280/bme280.c
            mcxn_drivers/spi_driver_mcxn947.c
            mcxn_drivers/gpio_driver_mcxn947.c
            mcxn_drivers/timer_driver_mcxn947.c
            mcxn_drivers/uart_driver_mcxn947.c
            sensors/bno_08x/sht_lib/euler.c
            sensors/bno_08x/sht_lib/sh2_SensorValue.c
            sensors/bno_08x/sht_lib/sh2_util.c
            sensors/bno_08x/sht_lib/sh2.c
            sensors/bno_08x/sht_lib/shtp.c
)

mcux_add_macro(
    "MCXN947=1"
    CC "-DPRINTF_FLOAT_ENABLE=1"
)


mcux_add_include(
    INCLUDES .
            mcxn_drivers/
            kalman_z/
            sensors/bme280/
            sensors/mpu_9250/
            sensors/bno_08x/
            sensors/bno_08x/sht_lib/
)
mcux_convert_binary(BINARY ${APPLICATION_BINARY_DIR}/${MCUX_SDK_PROJECT_NAME}.bin)

