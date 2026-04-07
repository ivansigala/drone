
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
            dshot.c
            rc_fsia6B.c
            dma_driver_mcxn947.c
            timer_driver_mcxn947.c
            pwm_driver_mcxn947.c
            uart_driver_mcxn947.c
)

mcux_add_macro(
    "MCXN947=1"
)


mcux_add_include(
    INCLUDES .
)
mcux_convert_binary(BINARY ${APPLICATION_BINARY_DIR}/${MCUX_SDK_PROJECT_NAME}.bin)

