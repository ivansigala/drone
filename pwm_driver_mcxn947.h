/*
 * PWM_DRIVER.h
 *
 *  Created on: Nov 21, 2025
 *      Author: diego
 */

#ifndef PWM_DRIVER_MCXN947_H_
#define PWM_DRIVER_MCXN947_H_

#include "app.h"
#include "fsl_port.h"
#include "fsl_pwm.h"
#include "fsl_common.h"

#define DEMO_PWM_DISABLE_MAP_OP
#define PWM_FAULT_LEVEL           true
#define PWM_SRC_CLK_FREQ          CLOCK_GetFreq(kCLOCK_BusClk)

/**
 * @brief Structure to hold the necessary hardware information for a single PWM channel.
 * @param pwm_base: Pointer to the PWM peripheral base address (e.g., PWM1)
 * @param submodule: The specific submodule (e.g., kPWM_Module_0)
 * @param channel: The specific channel (e.g., kPWM_PwmA, kPWM_PwmB)
 * @param pwm_mode: The PWM mode set during initialization (e.g., kPWM_SignedCenterAligned)
 * @param use_dma: Whether this channel should use DMA for updates (e.g., true or false)
 */
typedef struct _pwm_ctrl_t{
    PWM_Type *pwm_base;
    pwm_fault_input_t fault_level;
    pwm_module_control_t submodule_ctrl;
    pwm_submodule_t submodule; 
    pwm_channels_t channel;    
    pwm_mode_t pwm_mode;

    uint32_t frquency_u32;
    bool enable_dma;
} pwm_ctrl_t;

/* Get Default PWM Config for DShot Motors */
void pwm_get_default_dshot_config(pwm_ctrl_t *ctrl, uint8_t motor_id);

status_t pwm_init(pwm_ctrl_t *pwm);
status_t PWM_DRV_Init3PhPwm(pwm_ctrl_t *pwm);
void PWM_SetPwmDutyCycle(pwm_ctrl_t *pwm, uint16_t dutyCycle);
void pwm_set_ldok(pwm_ctrl_t *pwm);
void pwm_set_ldok_mask(PWM_Type *base, uint8_t mask);

#endif /* PWM_DRIVER_MCXN947_H_ */
