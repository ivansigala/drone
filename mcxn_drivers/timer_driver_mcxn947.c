/*
 * TIMER_DRIVER.c
 *
 *  Created on: Nov 22, 2025
 *      Author: diego
 */

#include "timer_driver_mcxn947.h"
#include "fsl_debug_console.h"

void (*callback_lptmr0)(void* args);
void (*callback_lptmr1)(void* args);

/*
 * timer_tick_count is incremented in LPTMR0_IRQHandler and read by
 * tasks. It MUST be volatile -- without it the compiler is free to
 * hoist the load out of a loop and the task will see a stale value
 * (very often it appears to be stuck at 0 under -O2).
 *
 * timer0_us_per_tick is set when LPTMR0 is initialised so the
 * timer_get_us / timer_get_ms helpers don't have to know the loop
 * frequency from the outside.
 */
static volatile uint32_t timer_tick_count    = 0;
static          uint32_t timer0_us_per_tick  = 0;

void delay_blocking_us(uint32_t microseconds){

    SDK_DelayAtLeastUs(microseconds, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);

}

uint32_t get_timer_tick_count(void){
    return timer_tick_count;
}

/*!
 * @brief Microseconds since LPTMR0 started ticking. Wraps at ~71 min.
 *
 *  Resolution is one LPTMR0 period (1000000 / timer_0.frequency), so
 *  for the current 400 Hz loop you get 2500 us per tick. Good enough
 *  for telemetry plotting; if you need sub-tick resolution we can
 *  add a CNR-based overlay (see notes in timer_driver_mcxn947.h).
 */
uint32_t timer_get_us(void) {
    return timer_tick_count * timer0_us_per_tick;
}

/*!
 * @brief Milliseconds since LPTMR0 started ticking. Wraps at ~49 days.
 */
uint32_t timer_get_ms(void) {
    /* Integer math: keep us first, then divide. With timer0_us_per_tick
     * = 2500 the reported ms steps are 0, 2, 5, 7, 10, 12, ... -- still
     * monotonic, just non-uniform 2/3 ms steps because 2.5 ms doesn't
     * divide evenly. Send timer_get_us() if that bothers you. */
    return (timer_tick_count * timer0_us_per_tick) / 1000U;
}

status_t timer_init(timer_ctrl_t *timer){

    lptmr_config_t lptmrConfig;
    uint32_t period_ticks;

    if(timer->timer_id == 0){
    	timer->lptmr_base = LPTMR0;
    } else if (timer->timer_id == 1){
    	timer->lptmr_base = LPTMR1;
    } else {
    	return kStatus_Fail;
    }

    /*Enable 12MHz clock*/
    CLOCK_SetupClockCtrl(kCLOCK_FRO12MHZ_ENA);

    lptmrConfig.timerMode = kLPTMR_TimerModeTimeCounter;
    lptmrConfig.pinSelect = kLPTMR_PinSelectInput_0;
    lptmrConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
    lptmrConfig.enableFreeRunning = false;
    lptmrConfig.bypassPrescaler = true;
    lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_0;
    lptmrConfig.value = kLPTMR_Prescale_Glitch_0;

    if( !(timer->lptmr_base == LPTMR0 || timer->lptmr_base == LPTMR1) ){
		return kStatus_Fail;
	}

    // Initialize the LPTMR
    LPTMR_Init(timer->lptmr_base, &lptmrConfig);

    // Calculate period_ticks from frequency (12MHz clock)
    period_ticks = CLOCK_SOURCE_LPTMR / timer->frequency;


    // Set timer period.
    LPTMR_SetTimerPeriod(timer->lptmr_base, period_ticks);

    /* Remember LPTMR0's period in microseconds so timer_get_us() /
     * timer_get_ms() can convert correctly without any caller having
     * to repeat the math. Only LPTMR0's tick is counted, so only its
     * frequency matters here. */
    if (timer->lptmr_base == LPTMR0) {
        timer0_us_per_tick = 1000000U / timer->frequency;
    }

    return kStatus_Success;
}

void timer_start(timer_ctrl_t *timer){

    LPTMR_StartTimer(timer->lptmr_base);
    
}

void timer_stop(timer_ctrl_t *timer){

    LPTMR_StopTimer(timer->lptmr_base);
}

void LPTMR0_IRQHandler(void)
{
    LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);

    /* Single producer of timer_tick_count; the variable is volatile so
     * the read side is coherent without any further barriers here. */
    timer_tick_count++;

    if(callback_lptmr0 != NULL){
        callback_lptmr0(NULL);
    }
    __DSB();
    __ISB();
}

void LPTMR1_IRQHandler(void)
{
    LPTMR_ClearStatusFlags(LPTMR1, kLPTMR_TimerCompareFlag);

    if(callback_lptmr1 != NULL){
    	callback_lptmr1(NULL);
    }

    __DSB();
    __ISB();
}


void timer_attach_callback(timer_ctrl_t *timer, void* callback){

	// Enable timer interrupt
	LPTMR_EnableInterrupts(timer->lptmr_base, kLPTMR_TimerInterruptEnable);

	// Enable at the NVIC
	if(timer->lptmr_base == LPTMR0){
        NVIC_SetPriority(LPTMR0_IRQn, 5);
		EnableIRQ(LPTMR0_IRQn);
	}else if(timer->lptmr_base == LPTMR1){
        NVIC_SetPriority(LPTMR1_IRQn, 5);
		EnableIRQ(LPTMR1_IRQn);
	} else {
		PRINTF("Not a valid lptmr base: %p", timer->lptmr_base );
		return;
	}

	if(timer->lptmr_base == LPTMR0){
		callback_lptmr0 = callback;
	} else {
		callback_lptmr1 = callback;
	}



}
