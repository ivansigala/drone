/*
 * dynamics.h
 *
 * Discrete Sliding-Mode Controller (SMC) for the quadrotor + attitude
 * kinematic helpers. The math layout mirrors the Simulink reference exactly,
 * so any tuning change in the MATLAB model can be ported by editing the
 * macros below.
 *
 * State vector convention (matches the MATLAB Simulink blocks 1-indexed):
 *   states[0] = x         states[6]  = roll
 *   states[1] = y         states[7]  = pitch
 *   states[2] = z         states[8]  = yaw
 *   states[3] = dx        states[9]  = droll
 *   states[4] = dy        states[10] = dpitch
 *   states[5] = dz        states[11] = dyaw
 *
 * Created on: Mayo, 2026
 * Author:     diego
 */

#ifndef DYNAMICS_H_
#define DYNAMICS_H_

#include "arm_math.h"

/* ----- Discretization ---------------------------------------------------- */
#ifndef DYNAMICS_T
#define DYNAMICS_T              (1.0f / 100.0f)     /* 10 ms / 100 Hz       */
#endif

/* ----- Physical constants (must match the Simulink Callback) ------------- */
#define GRAVITY                 9.81f               /* m/s^2                */
#define DRONE_M                 0.6f                /* kg                   */

#define CM                      3.5e-6f             /* kg*m^2  rotor inertia */
#define K1                      0.7426e-6f          /* N*m / (rad/s)^2       */
#define K2                      0.1485e-6f          /* N   / (rad/s)^2       */

/* Motor positions (body frame, m) */
#define MOTOR_1_X                0.12651f
#define MOTOR_1_Y                0.09313f
#define MOTOR_2_X               -0.12651f
#define MOTOR_2_Y               -0.09313f
#define MOTOR_3_X               -0.12651f
#define MOTOR_3_Y                0.09313f
#define MOTOR_4_X                0.12651f
#define MOTOR_4_Y               -0.09313f

/* Aerodynamic drag coefficients */
#define DRONE_SX                 0.0024f
#define DRONE_SY                 0.0072f
#define DRONE_SZ                 0.0072f

/* ----- SMC tuning -------------------------------------------------------- */
#define SMC_A_Z                  3.0f
#define SMC_XI_Z                 0.5f
#define SMC_E_Z                  0.05f
#define SMC_EPSILON_Z            0.1f

#define SMC_A_X                  3.0f
#define SMC_XI_X                 0.5f
#define SMC_E_X                  0.05f
#define SMC_EPSILON_X            0.1f

#define SMC_A_Y                  3.0f
#define SMC_XI_Y                 0.5f
#define SMC_E_Y                  0.05f
#define SMC_EPSILON_Y            0.1f

#define SMC_A_ETA                10.0f
#define SMC_XI_ETA               5.0f
#define SMC_E_ETA                0.5f
#define SMC_EPSILON_ETA          0.05f

/* Maximum commanded tilt angle (rad) passed to asin() by the
 * position controller; mirrors `max(min(arg, 0.2), -0.2)` in MATLAB. */
#ifndef SMC_TILT_MAX
#define SMC_TILT_MAX             0.2f
#endif

/* Numerical safety thresholds */
#ifndef DYNAMICS_EPS_DENOM
#define DYNAMICS_EPS_DENOM       1.0e-6f
#endif

/* ----- Inertia tensor (body frame, kg*m^2) ------------------------------- */
extern const float                  DRONE_J_DATA[9];
extern const arm_matrix_instance_f32 DRONE_J;

/* ========================================================================
 *                       KINEMATICS HELPERS
 * ========================================================================
 *
 * Lambda(roll, pitch)  maps Euler-rate -> body angular velocity
 *      Omega_body = Lambda * [droll; dpitch; dyaw]
 *
 * dLambda/dt          time derivative of Lambda for the current state.
 *
 * Trig of the current attitude must be refreshed once per tick via
 * dynamics_update_trig() before calling any of the controllers.
 * ======================================================================== */

void dynamics_update_trig   (float roll, float pitch, float yaw);
void dynamics_compute_lambda (float out[9]);
void dynamics_compute_dlambda(float droll, float dpitch, float out[9]);

/* ========================================================================
 *                       SMC CONTROLLER API
 * ======================================================================== */

/*!
 * @brief Compute total thrust U0 from the altitude (Z) SMC loop.
 *
 * Outer-loop control. Assumes a step reference (dz_k_d = dz_kp1_d = 0).
 *
 * @param states  12-state vector (see file header).
 * @param z_k_d   Desired altitude (m).
 * @return        Commanded total thrust in N (force in body-Z, world-projected
 *                via 1/(cos(pitch)*cos(roll))).
 */
float dynamics_compute_U0(const float states[12], float z_k_d);

/*!
 * @brief Compute desired roll/pitch from the X/Y position SMC loops.
 *
 * Outputs the attitude references that the inner attitude loop should
 * track to drive the quadrotor toward (x_k_d, y_k_d).
 *
 * @note Requires sin(yaw) != 0 (mathematical singularity at yaw = 0,
 *       k*pi). The function safely returns 0 commands when this is hit.
 *
 * @param states     12-state vector.
 * @param x_k_d      Desired X (m).
 * @param y_k_d      Desired Y (m).
 * @param w          Current motor speeds (rad/s), 4-vector.
 * @param[out] roll_k_d   Commanded roll (rad).
 * @param[out] pitch_k_d  Commanded pitch (rad).
 */
void dynamics_compute_position_control(const float states[12],
                                       float x_k_d, float y_k_d,
                                       const float w[4],
                                       float *roll_k_d, float *pitch_k_d);

/*!
 * @brief Compute body torques [U1, U2, U3] from the attitude SMC loop.
 *
 * Inner-loop control. Tracks the desired Euler-angle reference eta_k_d
 * using the Lagrangian-form mass matrix J = Lambda^T * I * Lambda.
 *
 * @param states    12-state vector.
 * @param eta_k_d   Desired [roll_d; pitch_d; yaw_d] (rad).
 * @param w         Current motor speeds (rad/s), 4-vector (for rotor
 *                  gyroscopic torque cancellation).
 * @param[out] U    [U1=roll, U2=pitch, U3=yaw] body torques.
 */
void dynamics_compute_attitude_control(const float states[12],
                                       const float eta_k_d[3],
                                       const float w[4],
                                       float U[3]);

#endif /* DYNAMICS_H_ */
