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

#endif /* TIMER_DRIVER_MCXN947_H_ */
