/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * HIL host runner for the discrete SMC drone controller.
 *
 * Wire protocol over LPUART4 (VCOM bridge, 115200 8N1):
 *   PC  -> MCU : 16 IEEE-754 singles (64 bytes, little-endian)
 *                [x,y,z, dx,dy,dz, roll,pitch,yaw, droll,dpitch,dyaw,
 *                 w0,w1,w2,w3]
 *   MCU -> PC  : 4  IEEE-754 singles (16 bytes, little-endian)
 *                [U0, U1, U2, U3]
 *
 * 25 Hz hardware timer wakes the task; the read is blocking, so the loop
 * paces against whatever cadence the host actually sends.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include <math.h>
#include <stdint.h>

#include "fsl_device_registers.h"
#include "fsl_lpuart.h"
#include "board.h"
#include "app.h"

#include "dynamics.h"
#include "timer_driver_mcxn947.h"

#define control_task_PRIORITY  (configMAX_PRIORITIES - 1)
#define HIL_UART               LPUART4

static void control_task(void *pvParameters);
TaskHandle_t controlTaskHandle = NULL;

typedef union {
    float   f[16];
    uint8_t b[16 * 4];
} hil_rx_packet_t;

typedef union {
    float   f[4];
    uint8_t b[4 * 4];
} hil_tx_packet_t;

void timer_0_callback(void *args)
{
    (void)args;
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(controlTaskHandle, &woken);
    portYIELD_FROM_ISR(woken);
}

int main(void)
{
    BOARD_InitHardware();

    timer_ctrl_t timer_0 = { .timer_id = 0, .frequency = 100 };   /* 100 Hz = 10 ms */
    timer_init(&timer_0);
    timer_attach_callback(&timer_0, timer_0_callback);

    if (xTaskCreate(control_task, "control_task",
                    configMINIMAL_STACK_SIZE + 512,
                    NULL, control_task_PRIORITY,
                    &controlTaskHandle) != pdPASS)
    {
        while (1) { __asm volatile ("nop"); }
    }

    timer_start(&timer_0);
    vTaskStartScheduler();
    for (;;) { }
}

static void control_task(void *pvParameters)
{
    (void)pvParameters;

    hil_rx_packet_t rx;
    hil_tx_packet_t tx;

    // const float x_target   = 1.0f;
    // const float y_target   = 1.0f;
    const float z_target   = 1.0f;
    const float roll_target  = 0.0f;
    const float pitch_target = 0.0f;
    const float yaw_target = 0.0f;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (LPUART_ReadBlocking(HIL_UART, rx.b, sizeof(rx.b)) != kStatus_Success) {
            continue;
        }

        const float *states = &rx.f[0];
        const float *w      = &rx.f[12];

        dynamics_update_trig(states[2], states[3], states[4]);

        float U0 = dynamics_compute_U0(states, z_target);

        // float roll_d  = roll_target;
        // float pitch_d = pitch_target;
        // dynamics_compute_position_control(states, x_target, y_target, w,
        //                                   &roll_d, &pitch_d);

        const float eta_d[3] = { roll_target, pitch_target, yaw_target };
        float U123[3];
        dynamics_compute_attitude_control(states, eta_d, w, U123);

        tx.f[0] = U0;
        tx.f[1] = U123[0];
        tx.f[2] = U123[1];
        tx.f[3] = U123[2];

        LPUART_WriteBlocking(HIL_UART, tx.b, sizeof(tx.b));
    }
}
