/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * HIL host runner for the discrete SMC drone controller.
 *
 * The Simulink plant was reduced (May 2026) to control only altitude and
 * the three Euler angles, so the host-side state vector shrank from 12
 * to 8 floats.
 *
 * Wire protocol over LPUART4 (VCOM bridge, 115200 8N1):
 *   PC  -> MCU : 12 IEEE-754 singles (48 bytes, little-endian)
 *                [z, dz, roll, pitch, yaw, droll, dpitch, dyaw,
 *                 w0, w1, w2, w3]
 *   MCU -> PC  :  4 IEEE-754 singles (16 bytes, little-endian)
 *                [U0, U1, U2, U3]
 *
 * 100 Hz hardware timer wakes the task; the read is blocking, so the loop
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

#define HIL_RX_FLOATS          12      /* 8 states + 4 motor speeds */
#define HIL_TX_FLOATS           4      /* U0, U1, U2, U3            */

static void control_task(void *pvParameters);
TaskHandle_t controlTaskHandle = NULL;

typedef union {
    float   f[HIL_RX_FLOATS];
    uint8_t b[HIL_RX_FLOATS * 4];
} hil_rx_packet_t;

typedef union {
    float   f[HIL_TX_FLOATS];
    uint8_t b[HIL_TX_FLOATS * 4];
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

    const float z_target     = 1.0f;
    const float roll_target  = 0.0f;
    const float pitch_target = 0.0f;
    const float yaw_target   = 0.0f;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (LPUART_ReadBlocking(HIL_UART, rx.b, sizeof(rx.b)) != kStatus_Success) {
            continue;
        }

        /* Layout:  rx.f[0..7] = [z, dz, roll, pitch, yaw, droll, dpitch, dyaw]
         *          rx.f[8..11] = [w0, w1, w2, w3]                              */
        const float *states = &rx.f[0];
        const float *w      = &rx.f[8];

        /* Refresh cached attitude trig (roll=2, pitch=3, yaw=4) */
        dynamics_update_trig(states[2], states[3], states[4]);

        float U0 = dynamics_compute_U0(states, z_target);

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
