/*
 * dynamics.h
 *
 * Discrete Sliding-Mode Controller (SMC) for the quadrotor + attitude
 * kinematic helpers. The math layout mirrors the Simulink reference exactly,
 * so any tuning change in the MATLAB model can be ported by editing the
 * macros below.
 *
 * NOTE (May 2026): the Simulink plant was reduced to control only altitude
 * (Z) and the three Euler angles. The state vector dropped from 12 to 8.
 *
 * State vector convention (matches the MATLAB Simulink blocks 1-indexed):
 *   states[0] = z         states[4] = yaw
 *   states[1] = dz        states[5] = droll
 *   states[2] = roll      states[6] = dpitch
 *   states[3] = pitch     states[7] = dyaw
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
#define DRONE_M                 0.8f                /* kg                   */

#define CM                      3.5e-6f             /* kg*m^2  rotor inertia */
#define K1                      0.6426e-6f          /* N*m / (rad/s)^2       */
#define K2                      2.52335e-6f         /* N   / (rad/s)^2       */

/* Aerodynamic drag coefficient (Z axis only — XY no longer controlled) */
#define DRONE_SZ                 0.0072f

#define DY1         0.0931f
#define DY2         -0.0931f
#define DY3         0.0931f
#define DY4         -0.0931f

#define DX1         0.1265f
#define DX2         -0.1265f
#define DX3         -0.1265f
#define DX4         -0.1265f

/* ----- SMC tuning -------------------------------------------------------- */
#define SMC_A_Z                  3.0f
#define SMC_XI_Z                 0.5f
#define SMC_E_Z                  0.05f
#define SMC_EPSILON_Z            0.1f

#define SMC_A_ETA                10.0f
#define SMC_XI_ETA               5.0f
#define SMC_E_ETA                0.5f
#define SMC_EPSILON_ETA          0.05f

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
 * @param states  8-state vector (see file header).
 * @param z_k_d   Desired altitude (m).
 * @return        Commanded total thrust in N (force in body-Z, world-projected
 *                via 1/(cos(pitch)*cos(roll))).
 */
float dynamics_compute_U0(const float states[8], float z_k_d);

/*!
 * @brief Compute body torques [U1, U2, U3] from the attitude SMC loop.
 *
 * Inner-loop control. Tracks the desired Euler-angle reference eta_k_d
 * using the Lagrangian-form mass matrix J = Lambda^T * I * Lambda.
 *
 * @param states    8-state vector.
 * @param eta_k_d   Desired [roll_d; pitch_d; yaw_d] (rad).
 * @param w         Current motor speeds (rad/s), 4-vector (for rotor
 *                  gyroscopic torque cancellation).
 * @param[out] U    [U1=roll, U2=pitch, U3=yaw] body torques.
 */
void dynamics_compute_attitude_control(const float states[8],
                                       const float eta_k_d[3],
                                       const float w[4],
                                       float U[3]);


void dynamics_compute_dw(float w[4], float U[4]);


#endif /* DYNAMICS_H_ */
