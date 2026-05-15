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

void delay_blocking_us(uint32_t microseconds){
    
    SDK_DelayAtLeastUs(microseconds, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);

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
