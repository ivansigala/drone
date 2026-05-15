/*
 * arm_controller.c
 *
 * Discrete two-block controller for a 2-DOF planar arm. Math is laid out
 * to match the MATLAB reference line-for-line; every named intermediate
 * here has the same role as the same-named variable in the .m file.
 *
 * Closed-form algebra notes
 * -------------------------
 *   C  = [1 -1 ; 1 1]        =>   inv(C) = 0.5 * [1 1 ; -1 1]
 *
 *   D  = [d11 d12 ; d12 d22] =>   inv(D) = (1/det) * [d22 -d12 ; -d12 d11]
 *
 *   B34_cont   = inv(D)
 *   B34d       = T * B34_cont = T * inv(D)
 *   inv(B34d)  = (1/T) * D
 *
 *   f34d_disc  = x34 + f34_cont * T          (open-loop one-step prediction)
 *
 * Created: May 2026
 * Author:  diego
 */

#include "arm_controller.h"
#include <math.h>

/* Local 2x1 helpers keep the control law readable. */
static inline void cd12_mul(float v1, float v2, float *y1, float *y2)
{
    /* y = C * v   with   C = [1 -1 ; 1 1] */
    *y1 = v1 - v2;
    *y2 = v1 + v2;
}

static inline void cd12_inv_mul(float a1, float a2, float *x1, float *x2)
{
    /* x = inv(C) * a   with   inv(C) = 0.5 * [1 1 ; -1 1] */
    *x1 = 0.5f * ( a1 + a2);
    *x2 = 0.5f * (-a1 + a2);
}

void arm_controller_step(const float state[4],
                         const float q_des_k[2],
                         const float q_des_kp1[2],
                         const float q_des_kp2[2],
                         float       u_out[2])
{
    const float q1  = state[0];
    const float q2  = state[1];
    const float dq1 = state[2];
    const float dq2 = state[3];

    /* ============================================================
     *  1. Continuous robot dynamics at (q, dq)
     * ============================================================ */
    const float c1   = cosf(q1);
    const float c2   = cosf(q2);
    const float s2   = sinf(q2);
    const float c1p2 = cosf(q1 + q2);

    /* Inertia matrix D */
    const float d11 = ARM_M1*ARM_LC1*ARM_LC1
                    + ARM_M2*(ARM_L1*ARM_L1 + ARM_LC2*ARM_LC2
                              + 2.0f*ARM_L1*ARM_LC2*c2)
                    + ARM_I1 + ARM_I2;
    const float d12 = ARM_M2*(ARM_LC2*ARM_LC2 + ARM_L1*ARM_LC2*c2) + ARM_I2;
    const float d22 = ARM_M2*ARM_LC2*ARM_LC2 + ARM_I2;

    /* Coriolis (Cv), gravity (G), viscous friction (Bv) */
    const float h    = -ARM_M2*ARM_L1*ARM_LC2*s2;
    const float phi1 =  (ARM_M1*ARM_LC1 + ARM_M2*ARM_L1)*c1;
    const float phi2 =   ARM_M2*ARM_LC2*c1p2;

    const float Cv1 =  (2.0f*dq1*dq2 + dq2*dq2)*h;
    const float Cv2 = -dq1*dq1*h;
    const float G1  =  ARM_GRAVITY*(phi1 + phi2);
    const float G2  =  ARM_GRAVITY*phi2;
    const float Bv1 =  ARM_B1*dq1;
    const float Bv2 =  ARM_B2*dq2;

    /* f34_cont = D \ (-Cv - G - Bv) */
    const float rhs1   = -Cv1 - G1 - Bv1;
    const float rhs2   = -Cv2 - G2 - Bv2;
    const float det_D  =  d11*d22 - d12*d12;
    const float inv_dD =  1.0f / det_D;
    const float f34_1  =  inv_dD * ( d22*rhs1 - d12*rhs2);
    const float f34_2  =  inv_dD * (-d12*rhs1 + d11*rhs2);

    /* Discrete pieces of the plant:
     *   f34d  = x34 + f34_cont*T       (open-loop one-step prediction)
     *   B34d  = T * inv(D)             (used only via inv(B34d) = D/T)         */
    const float f34d_1 = dq1 + f34_1 * ARM_T;
    const float f34d_2 = dq2 + f34_2 * ARM_T;

    /* ============================================================
     *  2. Convert joint references to OUTPUT space  r = C * q_des
     * ============================================================ */
    float r_k_1,   r_k_2;
    float r_kp1_1, r_kp1_2;
    float r_kp2_1, r_kp2_2;
    cd12_mul(q_des_k[0],   q_des_k[1],   &r_k_1,   &r_k_2);
    cd12_mul(q_des_kp1[0], q_des_kp1[1], &r_kp1_1, &r_kp1_2);
    cd12_mul(q_des_kp2[0], q_des_kp2[1], &r_kp2_1, &r_kp2_2);

    /* ============================================================
     *  3. BLOCK 1 — position error -> desired velocity x34r
     * ============================================================ */
    float y12_k_1, y12_k_2;
    cd12_mul(q1, q2, &y12_k_1, &y12_k_2);          /* y = C * x12 */

    const float e1_k_1 = r_k_1 - y12_k_1;
    const float e1_k_2 = r_k_2 - y12_k_2;

    /* x34r_k = (1/T) * inv(C) * ( r_{k+1} - C*x12 - K1 * e1 ) */
    const float arg1_1 = r_kp1_1 - y12_k_1 - ARM_K1_GAIN * e1_k_1;
    const float arg1_2 = r_kp1_2 - y12_k_2 - ARM_K1_GAIN * e1_k_2;

    const float inv_T = 1.0f / ARM_T;
    float x34r_k_1, x34r_k_2;
    cd12_inv_mul(arg1_1, arg1_2, &x34r_k_1, &x34r_k_2);
    x34r_k_1 *= inv_T;
    x34r_k_2 *= inv_T;

    /* Predict x12_{k+1} (= x12 + x34*T) and re-run Block 1 for x34r_{k+1} */
    const float x12n_1 = q1 + dq1 * ARM_T;
    const float x12n_2 = q2 + dq2 * ARM_T;

    float y12_kp1_1, y12_kp1_2;
    cd12_mul(x12n_1, x12n_2, &y12_kp1_1, &y12_kp1_2);

    const float e1_kp1_1 = r_kp1_1 - y12_kp1_1;
    const float e1_kp1_2 = r_kp1_2 - y12_kp1_2;

    const float arg2_1 = r_kp2_1 - y12_kp1_1 - ARM_K1_GAIN * e1_kp1_1;
    const float arg2_2 = r_kp2_2 - y12_kp1_2 - ARM_K1_GAIN * e1_kp1_2;

    float x34r_kp1_1, x34r_kp1_2;
    cd12_inv_mul(arg2_1, arg2_2, &x34r_kp1_1, &x34r_kp1_2);
    x34r_kp1_1 *= inv_T;
    x34r_kp1_2 *= inv_T;

    /* ============================================================
     *  4. BLOCK 2 — velocity error -> torque
     * ============================================================ */
    const float e2_k_1 = x34r_k_1 - dq1;
    const float e2_k_2 = x34r_k_2 - dq2;

    const float u_arg_1 = x34r_kp1_1 - f34d_1 - ARM_K2_GAIN * e2_k_1;
    const float u_arg_2 = x34r_kp1_2 - f34d_2 - ARM_K2_GAIN * e2_k_2;

    /* u = inv(B34d) * u_arg = (1/T) * D * u_arg */
    u_out[0] = inv_T * (d11 * u_arg_1 + d12 * u_arg_2);
    u_out[1] = inv_T * (d12 * u_arg_1 + d22 * u_arg_2);
}
