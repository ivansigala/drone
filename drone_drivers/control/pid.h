/*
 * pid.h
 *
 * Created on: Mayo, 2026
 * Author:     diego
 */

#ifndef PID_H_
#define PID_H_

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float integral_err;     /* running integral of error (rad·s) */
	float prev_measured;    /* previous shaft angle (rad) */
	float max_integral;     /* anti-windup clamps */
	float min_integral;
	float last_output;      /* normalised output last sample, for telemetry */
	float error_deadband;   /* errors whose |magnitude| falls at or below this
	                         * threshold are treated as zero.  Set to the
	                         * sensor's minimum resolvable step so the
	                         * integrator does not wind up chasing noise.
	                         * Use 0.0f to disable.                          */
} pid_state_t;

/*!
 * @brief Initialise a PID state struct with gains, integrator clamps, and
 *        an error deadband.
 *
 * @param error_deadband  Errors whose absolute value is at or below this
 *                        threshold are clamped to zero before any term is
 *                        computed.  The integral is not accumulated and the
 *                        P term is zero inside the band, but the D term
 *                        (derivative-on-measurement) still damps motion.
 *                        Pass 0.0f to disable the deadband entirely.
 */
void  pid_init(pid_state_t *pid,
               float Kp, float Ki, float Kd,
               float i_min, float i_max,
               float error_deadband);

/*!
 * @brief Run one iteration of the position PID.
 *
 * @param pid       PID state (gains + accumulators).
 * @param target    setpoint (rad).
 * @param measured  current shaft angle (rad).
 * @param dt        sample period (s) — should match the control task rate.
 * @return normalised output in [-1, 1]. Positive => forward, negative => reverse.
 */
float pid_compute(pid_state_t *pid, float target, float measured, float dt);




#endif /* PID_H_ */