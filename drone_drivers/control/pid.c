/*
 * pid.h
 *
 * Created on: Mayo, 2026
 * Author:     diego
 */

#include "pid.h"

void pid_init(pid_state_t *pid,
              float Kp, float Ki, float Kd,
              float i_min, float i_max,
              float error_deadband)
{
    pid->Kp             = Kp;
    pid->Ki             = Ki;
    pid->Kd             = Kd;
    pid->integral_err   = 0.0f;
    pid->prev_measured  = 0.0f;
    pid->min_integral   = i_min;
    pid->max_integral   = i_max;
    pid->last_output    = 0.0f;
    pid->error_deadband = (error_deadband < 0.0f) ? -error_deadband : error_deadband;
}


float pid_compute(pid_state_t *pid, float target, float measured, float dt)
{
    float error = target - measured;
    float output = 0;
    
    if(target <= 0.0f){
        /* Special case for zero target: skip the PID and just command
         * zero output.  This prevents any possibility of integral windup
         * or derivative spikes during motor spin-down, which can be a
         * long, low-speed transient that the ESC telemetry may report with
         * low resolution and high noise. */
        pid->integral_err = 0.0f;  /* reset integral to prevent windup during idle */
        pid->prev_measured = measured; /* reset derivative term to prevent spikes on re-arming */
        pid->last_output = 0.0f;
        return 0.0f;
    }

    /* Error deadband: if the raw error is within the sensor's resolution
     * floor, treat it as zero.  This stops the integrator from winding up
     * and the proportional term from reacting to noise that the ESC
     * telemetry cannot distinguish from a real speed difference.
     * The derivative (on measurement) is unaffected — it still damps any
     * real motion that happens to be below the threshold.               */
    float abs_err = (error < 0.0f) ? -error : error;
    if (abs_err <= pid->error_deadband)
        error = 0.0f;

    /* Derivative on MEASUREMENT (not error) avoids the derivative kick
     * when the setpoint changes step-wise: only the plant velocity
     * appears in the D term. */
    float d_meas = (measured - pid->prev_measured) / dt;

    /* Tentative output before updating the integrator. */
    output = pid->Kp * error
                 + pid->Ki * pid->integral_err
                 - pid->Kd * d_meas;

    /* Conditional integration: only accumulate if we're not already
     * pinned at saturation in the same direction as the current error.
     * Cheap, effective anti-windup. */
    bool sat_high = (output >=  1.0f) && (error > 0.0f);
    bool sat_low  = (output <= -1.0f) && (error < 0.0f);
    if (!sat_high && !sat_low) {
        pid->integral_err += error * dt;
        if (pid->integral_err > pid->max_integral) pid->integral_err = pid->max_integral;
        if (pid->integral_err < pid->min_integral) pid->integral_err = pid->min_integral;
    }

    /* Recompute with the (possibly) updated integrator. */
    output = pid->Kp * error
           + pid->Ki * pid->integral_err
           - pid->Kd * d_meas;

    /* Final clamp into normalised range. */
    if (output >  1.0f) output =  1.0f;
    if (output < -1.0f) output = -1.0f;

    pid->prev_measured = measured;
    pid->last_output   = output;

    return output;
}
