/*
 * dynamics.c
 *
 * Discrete SMC controllers + attitude kinematic helpers.
 * Math layout mirrors the Simulink reference 1:1.
 *
 * Created on: Mayo, 2026
 * Author:     diego
 */

#include "dynamics.h"
#include <math.h>

/* =========================================================================
 *      Constant tables (flash-resident)
 * ========================================================================= */

const float DRONE_J_DATA[9] = {
    0.0038f, 0.0001f, 0.0000f,
    0.0001f, 0.0056f, 0.0000f,
    0.0000f, 0.0000f, 0.0055f,
};

const arm_matrix_instance_f32 DRONE_J = {
    .numRows = 3,
    .numCols = 3,
    .pData   = (float32_t *)DRONE_J_DATA,
};

/* =========================================================================
 *      Cached attitude trig (refreshed each tick via dynamics_update_trig)
 * ========================================================================= */

static float s_roll;
static float s_pitch;
static float s_yaw;

static float s_sin_roll,  s_cos_roll;
static float s_sin_pitch, s_cos_pitch;
static float s_sin_yaw,   s_cos_yaw;

/* =========================================================================
 *      Private helpers
 * ========================================================================= */

static inline float signf(float x)
{
    return (float)((x > 0.0f) - (x < 0.0f));
}

/* Smoothed sign within +/- epsilon boundary layer; falls back to crisp
 * sign() if the boundary layer is non-positive. */
static inline float satf(float s, float epsilon)
{
    if (epsilon <= 0.0f) return signf(s);
    const float v = s / epsilon;
    if (v >  1.0f) return  1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

static inline void cross3(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static inline float clampf(float x, float lo, float hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

/* =========================================================================
 *      Kinematics
 * ========================================================================= */

void dynamics_update_trig(float roll, float pitch, float yaw)
{
    s_roll       = roll;
    s_pitch      = pitch;
    s_yaw        = yaw;

    s_sin_roll   = arm_sin_f32(roll);
    s_cos_roll   = arm_cos_f32(roll);
    s_sin_pitch  = arm_sin_f32(pitch);
    s_cos_pitch  = arm_cos_f32(pitch);
    s_sin_yaw    = arm_sin_f32(yaw);
    s_cos_yaw    = arm_cos_f32(yaw);
}

void dynamics_compute_lambda(float out[9])
{
    /* Lambda(roll, pitch) — Euler-rate -> body angular velocity */
    out[0] = 1.0f; out[1] = 0.0f;         out[2] = -s_sin_pitch;
    out[3] = 0.0f; out[4] =  s_cos_roll;  out[5] =  s_sin_roll * s_cos_pitch;
    out[6] = 0.0f; out[7] = -s_sin_roll;  out[8] =  s_cos_roll * s_cos_pitch;
}

void dynamics_compute_dlambda(float droll, float dpitch, float out[9])
{
    /* dLambda/dt evaluated with the *current* Euler rates (droll, dpitch).
     * Matches the Simulink expression element-by-element. */
    out[0] = 0.0f;
    out[1] = 0.0f;
    out[2] = -dpitch * s_cos_pitch;

    out[3] = 0.0f;
    out[4] = -droll * s_sin_roll;
    out[5] =  droll  * s_cos_pitch * s_cos_roll
           -  dpitch * s_sin_pitch * s_sin_roll;

    out[6] = 0.0f;
    out[7] = -droll * s_cos_roll;
    out[8] = -droll  * s_cos_pitch * s_sin_roll
           -  dpitch * s_cos_roll  * s_sin_pitch;
}

/* =========================================================================
 *      SMC #1 — Altitude controller (outputs U0)
 *
 *   z_{k+1}_d = z_k_d, dz_k_d = dz_{k+1}_d = 0
 *
 *   A      = (T/M) * ( -M*g - Sz*sign(dz)*dz^2 )
 *   s_z    = a_z*(z_k_d - z_k) - dz_k
 *   U0     = (M / (cos(pitch)*cos(roll)*T))
 *              * ( -a_z*T*dz - A + xi_z*T*s_z + e_z*T*sat(s_z) )
 * ========================================================================= */

float dynamics_compute_U0(const float states[12], float z_k_d)
{
    const float z_k  = states[2];
    const float dz_k = states[5];

    const float A = (DYNAMICS_T / DRONE_M) *
                    (-DRONE_M * GRAVITY
                     - DRONE_SZ * signf(dz_k) * dz_k * dz_k);

    const float s_z   = SMC_A_Z * (z_k_d - z_k) - dz_k;
    const float sat_s = satf(s_z, SMC_EPSILON_Z);

    const float numer = -SMC_A_Z  * DYNAMICS_T * dz_k
                      -  A
                      +  SMC_XI_Z * DYNAMICS_T * s_z
                      +  SMC_E_Z  * DYNAMICS_T * sat_s;

    const float denom = s_cos_pitch * s_cos_roll * DYNAMICS_T;
    if (fabsf(denom) < DYNAMICS_EPS_DENOM) {
        return 0.0f;                               /* gimbal-lock safety */
    }
    return (DRONE_M / denom) * numer;
}

/* =========================================================================
 *      SMC #2 — Position controller (outputs roll_d, pitch_d)
 *
 *   U0  = k2 * sum_i w_i*|w_i|       (current achieved thrust)
 *   D   = T * U0 * sin(yaw)
 *
 *   C   = (T*U0/M)*cos(yaw)*sin(pitch) - (T/M)*Sx*sign(dx)*dx^2
 *   s_x = a_x*(x_k_d - x_k) - dx_k
 *   roll_arg  = (M/D)*( -a_x*T*dx - C + xi_x*T*s_x + e_x*T*sat(s_x) )
 *
 *   E   = -(T*U0/M)*cos(yaw)*sin(roll) - (T/M)*Sy*sign(dy)*dy^2
 *   s_y = a_y*(y_k_d - y_k) - dy_k
 *   pitch_arg = (M/D)*( -a_y*T*dy - E + xi_y*T*s_y + e_y*T*sat(s_y) )
 *
 *   roll_d  = asin(clamp(roll_arg,  +/-SMC_TILT_MAX))
 *   pitch_d = asin(clamp(pitch_arg, +/-SMC_TILT_MAX))
 * ========================================================================= */

void dynamics_compute_position_control(const float states[12],
                                       float x_k_d, float y_k_d,
                                       const float w[4],
                                       float *roll_k_d, float *pitch_k_d)
{
    const float x_k  = states[0];
    const float y_k  = states[1];
    const float dx_k = states[3];
    const float dy_k = states[4];

    /* Current rotor-produced thrust. Sign-preserving square keeps the
     * physics consistent if a motor ever spins negative. */
    const float U0 = K2 * ( fabsf(w[0]) * w[0]
                          + fabsf(w[1]) * w[1]
                          + fabsf(w[2]) * w[2]
                          + fabsf(w[3]) * w[3] );

    const float D  = DYNAMICS_T * U0 * s_sin_yaw;

    /* Singularity at yaw = k*pi: roll has no influence on x — bail safely */
    if (fabsf(U0) < DYNAMICS_EPS_DENOM ||
        fabsf(D)  < DYNAMICS_EPS_DENOM) {
        *roll_k_d  = 0.0f;
        *pitch_k_d = 0.0f;
        return;
    }

    const float M_over_D = DRONE_M / D;
    const float T_over_M = DYNAMICS_T / DRONE_M;
    const float TU0_M    = DYNAMICS_T * U0 / DRONE_M;

    /* ---- X axis -> roll_d ---- */
    const float C = TU0_M * s_cos_yaw * s_sin_pitch
                  - T_over_M * DRONE_SX * signf(dx_k) * dx_k * dx_k;

    const float s_x   = SMC_A_X * (x_k_d - x_k) - dx_k;
    const float sat_x = satf(s_x, SMC_EPSILON_X);

    float roll_arg = M_over_D *
                     ( -SMC_A_X  * DYNAMICS_T * dx_k
                       -  C
                       +  SMC_XI_X * DYNAMICS_T * s_x
                       +  SMC_E_X  * DYNAMICS_T * sat_x );

    roll_arg = clampf(roll_arg, -SMC_TILT_MAX, SMC_TILT_MAX);
    *roll_k_d = asinf(roll_arg);

    /* ---- Y axis -> pitch_d ---- */
    const float E = -TU0_M * s_cos_yaw * s_sin_roll
                  -  T_over_M * DRONE_SY * signf(dy_k) * dy_k * dy_k;

    const float s_y   = SMC_A_Y * (y_k_d - y_k) - dy_k;
    const float sat_y = satf(s_y, SMC_EPSILON_Y);

    float pitch_arg = M_over_D *
                      ( -SMC_A_Y  * DYNAMICS_T * dy_k
                        -  E
                        +  SMC_XI_Y * DYNAMICS_T * s_y
                        +  SMC_E_Y  * DYNAMICS_T * sat_y );

    pitch_arg = clampf(pitch_arg, -SMC_TILT_MAX, SMC_TILT_MAX);
    *pitch_k_d = asinf(pitch_arg);
}

/* =========================================================================
 *      SMC #3 — Attitude controller (outputs U1, U2, U3)
 *
 *   eta   = [roll;  pitch;  yaw]
 *   deta  = [droll; dpitch; dyaw]
 *
 *   J     = Lambda^T * I * Lambda          (Euler-rate mass matrix)
 *   Omega = Lambda * deta                  (body angular velocity)
 *   alpha = [0; 0; Cm * sum(w)]            (rotor gyroscopic torque)
 *
 *   B     = -J*dLambda*deta - cross(Omega, J*Omega) - cross(Omega, alpha)
 *
 *   s_eta = a_eta*(eta_k_d - eta) - deta
 *   V     = -a_eta*T*deta + e_eta*T*sat(s_eta) + xi_eta*T*s_eta
 *   U     = (J*Lambda / T) * V  -  B
 * ========================================================================= */

void dynamics_compute_attitude_control(const float states[12],
                                       const float eta_k_d[3],
                                       const float w[4],
                                       float U[3])
{
    const float eta_k[3]  = { states[6], states[7], states[8]  };
    float       deta_k[3] = { states[9], states[10], states[11] };

    /* ---- Build Lambda and dLambda from cached trig ---- */
    float lambda_data[9];
    float dlambda_data[9];
    dynamics_compute_lambda(lambda_data);
    dynamics_compute_dlambda(deta_k[0], deta_k[1], dlambda_data);

    arm_matrix_instance_f32 Lambda  = { 3, 3, lambda_data  };
    arm_matrix_instance_f32 dLambda = { 3, 3, dlambda_data };
    arm_matrix_instance_f32 I_mat   = { 3, 3, (float32_t *)DRONE_J_DATA };

    /* ---- J = Lambda^T * I * Lambda and J_Lambda = J * Lambda ---- */
    float lambda_T_data[9], tmp_data[9], J_data[9], JL_data[9];
    arm_matrix_instance_f32 Lambda_T = { 3, 3, lambda_T_data };
    arm_matrix_instance_f32 Tmp      = { 3, 3, tmp_data      };
    arm_matrix_instance_f32 J_mat    = { 3, 3, J_data        };
    arm_matrix_instance_f32 J_Lambda = { 3, 3, JL_data       };

    arm_mat_trans_f32(&Lambda,   &Lambda_T);
    arm_mat_mult_f32 (&Lambda_T, &I_mat,   &Tmp);
    arm_mat_mult_f32 (&Tmp,      &Lambda,  &J_mat);
    arm_mat_mult_f32 (&J_mat,    &Lambda,  &J_Lambda);

    /* ---- Sliding surface per Euler axis ---- */
    float s_eta[3], sat_s[3];
    for (int i = 0; i < 3; ++i) {
        s_eta[i] = SMC_A_ETA * (eta_k_d[i] - eta_k[i]) - deta_k[i];
        sat_s[i] = satf(s_eta[i], SMC_EPSILON_ETA);
    }

    /* ---- Nonlinear drift term  B  ---- */
    arm_matrix_instance_f32 deta_v = { 3, 1, deta_k };

    /* term1 = J * dLambda * deta */
    float v_dL[3], term1[3];
    arm_matrix_instance_f32 vdL_v = { 3, 1, v_dL  };
    arm_matrix_instance_f32 t1_v  = { 3, 1, term1 };
    arm_mat_mult_f32(&dLambda, &deta_v, &vdL_v);
    arm_mat_mult_f32(&J_mat,   &vdL_v,  &t1_v);

    /* Omega = Lambda * deta  ;  JOmega = J * Omega */
    float Omega[3], JOmega[3];
    arm_matrix_instance_f32 om_v  = { 3, 1, Omega  };
    arm_matrix_instance_f32 jom_v = { 3, 1, JOmega };
    arm_mat_mult_f32(&Lambda, &deta_v, &om_v);
    arm_mat_mult_f32(&J_mat,  &om_v,   &jom_v);

    /* alpha = [0; 0; Cm * sum(w)] — rotor gyroscopic torque */
    const float w_sum = w[0] + w[1] + w[2] + w[3];
    const float alpha[3] = { 0.0f, 0.0f, CM * w_sum };

    float term2[3], term3[3];
    cross3(Omega, JOmega, term2);
    cross3(Omega, alpha,  term3);

    /* ---- Build the SMC control law ---- */
    float V[3];
    for (int i = 0; i < 3; ++i) {
        V[i] = -SMC_A_ETA  * DYNAMICS_T * deta_k[i]
             +  SMC_E_ETA  * DYNAMICS_T * sat_s[i]
             +  SMC_XI_ETA * DYNAMICS_T * s_eta[i];
    }

    float JL_V[3];
    arm_matrix_instance_f32 V_v   = { 3, 1, V    };
    arm_matrix_instance_f32 JLV_v = { 3, 1, JL_V };
    arm_mat_mult_f32(&J_Lambda, &V_v, &JLV_v);

    /*    U = (J*Lambda / T) * V  -  B
     *      = (J*Lambda / T) * V  +  term1 + term2 + term3       */
    const float inv_T = 1.0f / DYNAMICS_T;
    for (int i = 0; i < 3; ++i) {
        U[i] = JL_V[i] * inv_T + term1[i] + term2[i] + term3[i];
    }
}
