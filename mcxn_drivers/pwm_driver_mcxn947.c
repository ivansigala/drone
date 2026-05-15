/*
 * PWM_DRIVER.c
 *
 *  Created on: Nov 21, 2025
 *      Author: diego
 */

#include "pwm_driver_mcxn947.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Chip Specific Defines (Moved from DShot header)
 ******************************************************************************/
#define DSHOT_PWM_BASE        PWM1
#define DSHOT_PWM_FREQ        600000U // DSHOT600

/* Motor Hardware Mapping (PWM) */
#define M0_PWM_SUBMODULE      kPWM_Module_0
#define M0_PWM_CTRL_MODULE    kPWM_Control_Module_0
#define M0_PWM_FAULT_LVL      kPWM_Fault_0

#define M1_PWM_SUBMODULE      kPWM_Module_1
#define M1_PWM_CTRL_MODULE    kPWM_Control_Module_1
#define M1_PWM_FAULT_LVL      kPWM_Fault_1

#define M2_PWM_SUBMODULE      kPWM_Module_2
#define M2_PWM_CTRL_MODULE    kPWM_Control_Module_2
#define M2_PWM_FAULT_LVL      kPWM_Fault_2

#define M3_PWM_SUBMODULE      kPWM_Module_3
#define M3_PWM_CTRL_MODULE    kPWM_Control_Module_3
#define M3_PWM_FAULT_LVL      kPWM_Fault_3


void pwm_get_default_dshot_config(pwm_ctrl_t *ctrl, uint8_t motor_id) {
    ctrl->pwm_base     = DSHOT_PWM_BASE;
    ctrl->pwm_mode     = kPWM_EdgeAligned;
    ctrl->frquency_u32 = DSHOT_PWM_FREQ;
    ctrl->enable_dma   = true;

    switch (motor_id) {
        case 0:
            ctrl->submodule    = M0_PWM_SUBMODULE;
            ctrl->submodule_ctrl = M0_PWM_CTRL_MODULE;
            ctrl->fault_level  = M0_PWM_FAULT_LVL;
            ctrl->channel      = kPWM_PwmA;
            break;
        case 1:
            ctrl->submodule    = M1_PWM_SUBMODULE;
            ctrl->submodule_ctrl = M1_PWM_CTRL_MODULE;
            ctrl->fault_level  = M1_PWM_FAULT_LVL;
            ctrl->channel      = kPWM_PwmA;
            break;
        case 2:
            ctrl->submodule    = M2_PWM_SUBMODULE;
            ctrl->submodule_ctrl = M2_PWM_CTRL_MODULE;
            ctrl->fault_level  = M2_PWM_FAULT_LVL;
            ctrl->channel      = kPWM_PwmA;
            break;
        case 3:
            ctrl->submodule    = M3_PWM_SUBMODULE;
            ctrl->submodule_ctrl = M3_PWM_CTRL_MODULE;
            ctrl->fault_level  = M3_PWM_FAULT_LVL;
            ctrl->channel      = kPWM_PwmA;
            break;
        default:
            break;
    }
}

status_t PWM_DRV_Init3PhPwm(pwm_ctrl_t *pwm)
{
    pwm_signal_param_t pwmSignals[2];
    uint32_t pwmSourceClockInHz;
    
    /* 1. Set standard ESC frequency */
    uint32_t pwmFrequencyInHz = pwm->frquency_u32;

    pwmSourceClockInHz = PWM_SRC_CLK_FREQ;

    /* 2. Configure Channel A */
    pwmSignals[0].pwmChannel       = kPWM_PwmA;
    pwmSignals[0].level            = kPWM_HighTrue;
    pwmSignals[0].dutyCyclePercent = 0; /* 0 percent dutycycle */
    pwmSignals[0].deadtimeValue    = 0; /* No deadtime needed for ESC data line */
    pwmSignals[0].faultState       = kPWM_PwmFaultState0;
    pwmSignals[0].pwmchannelenable = true;

    /* 3. Configure Channel B (Required for your pwm4 pin mapping) */
    pwmSignals[1].pwmChannel       = kPWM_PwmB;
    pwmSignals[1].level            = kPWM_HighTrue;
    pwmSignals[1].dutyCyclePercent = 0; 
    pwmSignals[1].deadtimeValue    = 0; 
    pwmSignals[1].faultState       = kPWM_PwmFaultState0;
    pwmSignals[1].pwmchannelenable = true;

    /*********** Configure all 4 submodules with BOTH channels (A & B) ************/
    return PWM_SetupPwm(pwm->pwm_base, pwm->submodule, pwmSignals, 2, pwm->pwm_mode, pwmFrequencyInHz, pwmSourceClockInHz);

}

status_t pwm_init(pwm_ctrl_t *pwm){

	pwm_config_t pwmConfig;
	pwm_fault_param_t faultConfig;


	PWM_GetDefaultConfig(&pwmConfig);

	/* Use full cycle reload */
	pwmConfig.reloadLogic = kPWM_ReloadPwmFullCycle;
	pwmConfig.enableDebugMode = true;
	pwmConfig.prescale = kPWM_Prescale_Divide_1;

	/* Initialize submodule 0 */
	if (PWM_Init(pwm->pwm_base, pwm->submodule, &pwmConfig) == kStatus_Fail)
	{
		PRINTF("PWM initialization failed\n");
		return kStatus_Fail;
	}


	PWM_FaultDefaultConfig(&faultConfig);


	faultConfig.faultLevel = PWM_FAULT_LEVEL;

	/* Sets up the PWM fault protection */
	PWM_SetupFaults(pwm->pwm_base, pwm->fault_level, &faultConfig);

	/* Set PWM fault disable mapping for submodule 0/1/2/3 */
	PWM_SetupFaultDisableMap(pwm->pwm_base, pwm->submodule, pwm->channel, kPWM_faultchannel_0,
							 DEMO_PWM_DISABLE_MAP_OP(kPWM_FaultDisable_0 | kPWM_FaultDisable_1 | kPWM_FaultDisable_2 | kPWM_FaultDisable_3));
	/*
	 * Call the init function with demo configuration.
	 * Recommend to invoke API PWM_SetupPwm after PWM and fault configuration, because reference manual advises to
	 * set OUTEN register after other PWM configurations. But set OUTEN register before MCTRL register is okay.
	 */
	if(PWM_DRV_Init3PhPwm(pwm) == kStatus_Fail)
	{
			PRINTF("PWM setup failed\n");
			return kStatus_Fail;
	}

	/* Set the load okay bit for all submodules to load registers from their buffer */
	PWM_SetPwmLdok(pwm->pwm_base, pwm->submodule_ctrl , true);

	/* Start the PWM generation from Submodules */
	//PWM_StartTimer(pwm->pwm_base, pwm->submodule_ctrl);

	if (pwm->enable_dma) {
        // Enable DMA write requests for the VAL registers
        // This triggers a DMA transfer at the start of every PWM reload
        PWM_EnableDMAWrite(pwm->pwm_base, pwm->submodule, true);
    }

	return kStatus_Success;

}


/*!
 * @brief Updates the duty cycle for a specific motor's PWM channel.
 * @param pwm Pointer to the PWM control structure to update.
 * @param dutyCycle The new duty cycle value to set.
 */
void PWM_SetPwmDutyCycle(pwm_ctrl_t *pwm, uint16_t dutyCycle)
{
    // The core logic is: update the duty cycle, then set the Load Okay (LDOK) bit.

	/*
	PWM_UpdatePwmDutycycle(pwm->pwm_base,
                           pwm->submodule,
                           pwm->channel,
                           kPWM_SignedCenterAligned, // Fixed mode from your original application
                           dutyCyclePercent);
	*/
	PWM_UpdatePwmDutycycleHighAccuracy(pwm->pwm_base,
			                           pwm->submodule, \
									   pwm->channel,
									   pwm->pwm_mode,
									   dutyCycle);
    // 2. Set the Load Okay (LDOK) bit for the specific submodule.
    // kPWM_Control_Module_X is equivalent to (1U << kPWM_Module_X).
    PWM_SetPwmLdok(pwm->pwm_base, (uint8_t)(1U << pwm->submodule), true);

}

/*!
 * @brief Activa el bit LDOK (Load Okay) para aplicar los nuevos valores en el siguiente ciclo PWM.
 * @param pwm Puntero a la estructura de control del PWM.
 */
void pwm_set_ldok(pwm_ctrl_t *pwm)
{
    // Usamos fsl_pwm internamente, ocultándolo de las capas superiores
    PWM_SetPwmLdok(pwm->pwm_base, (1U << pwm->submodule), true);
}

/*!
 * @brief Activa el bit LDOK (Load Okay) para una máscara de submódulos.
 * @param base Puntero al periférico PWM.
 * @param mask Máscara de los submódulos a activar.
 */
void pwm_set_ldok_mask(PWM_Type *base, uint8_t mask)
{
    PWM_SetPwmLdok(base, mask, true);
}

void pwm_init_timers(PWM_Type *base) {
    
    PWM_StartTimer(base, kPWM_Control_Module_0 | kPWM_Control_Module_1 | kPWM_Control_Module_2 | kPWM_Control_Module_3);


}



