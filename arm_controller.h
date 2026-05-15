/*
 * arm_controller.h
 *
 * Discrete two-block controller for a 2-DOF planar robot arm.
 *
 *   Block 1: position error in OUTPUT space  ->  desired velocity x34r
 *   Block 2: velocity error  ->  joint torque u
 *
 * Output matrix
 *      y = C * [q1; q2]   with   C = [ 1 -1 ;
 *                                      1  1 ]
 * is hard-coded; its inverse is 0.5 * [1 1; -1 1].
 *
 * Plant parameters and gains mirror the MATLAB reference 1:1 so a tuning
 * change in MATLAB only needs the macros below updated.
 *
 * State vector convention:
 *   state[0] = q1   [rad]
 *   state[1] = q2   [rad]
 *   state[2] = dq1  [rad/s]
 *   state[3] = dq2  [rad/s]
 *
 * Created: May 2026
 * Author:  diego
 */

#ifndef ARM_CONTROLLER_H_
#define ARM_CONTROLLER_H_

#include <stdint.h>

/* ----- Sampling period -------------------------------------------------- */
#ifndef ARM_T
#define ARM_T            (0.05f)        /* 50 ms / 20 Hz */
#endif

/* ----- Block-controller gains (|K1|<1, |K2|<1) -------------------------- */
#ifndef ARM_K1_GAIN
#define ARM_K1_GAIN      (0.7f)         /* position-error decay */
#endif
#ifndef ARM_K2_GAIN
#define ARM_K2_GAIN      (0.7f)         /* velocity-error decay */
#endif

/* ----- Robot parameters (must match the MATLAB plant exactly) ----------- */
#define ARM_L1           (2.0f)
#define ARM_L2           (2.0f)
#define ARM_LC1          (ARM_L1 * 0.5f)
#define ARM_LC2          (ARM_L2 * 0.5f)
#define ARM_M1           (0.5f)
#define ARM_M2           (0.5f)
#define ARM_B1           (0.3f)
#define ARM_B2           (0.3f)
#define ARM_I1           (0.05f)
#define ARM_I2           (0.05f)
#define ARM_GRAVITY      (9.81f)

/* ========================================================================
 *                    BLOCK CONTROLLER API
 * ========================================================================
 *
 * One discrete step of the two-block controller.
 *
 * Inputs
 *   state       Current plant state [q1, q2, dq1, dq2].
 *   q_des_k     Joint reference at step k.
 *   q_des_kp1   Joint reference at step k+1.
 *   q_des_kp2   Joint reference at step k+2.
 *
 * Output
 *   u_out       Joint torques [tau1, tau2].
 *
 * The references are JOINT-space; the controller converts them to the
 * output space y = C*[q1;q2] internally.
 * ======================================================================== */
void arm_controller_step(const float state[4],
                         const float q_des_k[2],
                         const float q_des_kp1[2],
                         const float q_des_kp2[2],
                         float       u_out[2]);

#endif /* ARM_CONTROLLER_H_ */
