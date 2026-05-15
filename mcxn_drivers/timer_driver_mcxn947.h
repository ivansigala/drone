/*
 * TIMER_DRIVER.h
 *
 *  Created on: Nov 22, 2025
 *      Author: diego
 */

#ifndef TIMER_DRIVER_MCXN947_H_
#define TIMER_DRIVER_MCXN947_H_

#include "fsl_common.h"
#include "fsl_lptmr.h"
//#include "fsl_ctimer.h"

#define CLOCK_SOURCE_LPTMR 12000000U

typedef struct timer_s {
    LPTMR_Type *lptmr_base;
    uint32_t frequency;
    uint32_t timer_id;
} timer_ctrl_t;

status_t timer_init(timer_ctrl_t *timer);
void timer_attach_callback(timer_ctrl_t *timer, void* callback);

void timer_start(timer_ctrl_t *timer);
void timer_stop(timer_ctrl_t *timer);

void delay_blocking_us(uint32_t microseconds);

uint32_t get_timer_tick_count(void);

/*!
 * @brief Microseconds since LPTMR0 started ticking.
 *
 *  Resolution = 1000000 / LPTMR0_frequency_hz (e.g. 2500 us at 400 Hz).
 *  Wraps as a uint32_t at ~71 minutes.
 */
uint32_t timer_get_us(void);

/*!
 * @brief Milliseconds since LPTMR0 started ticking. Wraps at ~49 days.
 *
 *  Use this for debug-UART timestamps. Pure integer math; safe to call
 *  from any context (the underlying tick counter is volatile and the
 *  per-tick period is set once in timer_init).
 */
uint32_t timer_get_ms(void);

#endif /* TIMER_DRIVER_MCXN947_H_ */
