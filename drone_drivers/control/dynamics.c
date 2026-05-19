/*
 * dynamics.c
 *
 * Discrete SMC controllers + attitude kinematic helpers.
 * Math layout mirrors the Simulink reference 1:1.
 *
 * State vector (8 elements, 0-indexed C / 1-indexed MATLAB):
 *   states[0]=z      states[4]=yaw
 *   states[1]=dz     states[5]=droll
 *   states[2]=roll   states[6]=dpitch
 *   states[3]=pitch  states[7]=dyaw
 *
 * Created on: Mayo, 2026
 * Author:     diego
 */

#include "dynamics.h"
#include <math.h>

#ifndef GLOBAL_Q
#define GLOBAL_Q 24
#endif

typedef int64_t _iq;

#define _IQ(x)       ((_iq)((double)(x) * (double)(1LL << GLOBAL_Q)))
#define _IQtoF(x)    ((float)((double)(x) / (double)(1LL << GLOBAL_Q)))
#define _IQmpy(A, B) (((A) * (B)) >> GLOBAL_Q)
#define _IQdiv(A, B) (((A) << GLOBAL_Q) / (B))
#define _IQabs(x)    ((x) < 0 ? -(x) : (x))

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

static _iq s_roll;
static _iq s_pitch;
static _iq s_yaw;

static _iq s_sin_roll,  s_cos_roll;
static _iq s_sin_pitch, s_cos_pitch;
static _iq s_sin_yaw,   s_cos_yaw;


/* =========================================================================
 *      Private helpers
 * ========================================================================= */

static inline _iq iq_sign(_iq x)
{
    return (x > 0) ? _IQ(1.0f) : ((x < 0) ? _IQ(-1.0f) : 0);
}

static inline _iq iq_sat(_iq s, _iq epsilon)
{
    if (epsilon <= 0) return iq_sign(s);
    _iq v = _IQdiv(s, epsilon);
    _iq one = _IQ(1.0f);
    if (v > one) return one;
    if (v < -one) return -one;
    return v;
}

static inline void iq_cross3(const _iq a[3], const _iq b[3], _iq out[3])
{
    out[0] = _IQmpy(a[1], b[2]) - _IQmpy(a[2], b[1]);
    out[1] = _IQmpy(a[2], b[0]) - _IQmpy(a[0], b[2]);
    out[2] = _IQmpy(a[0], b[1]) - _IQmpy(a[1], b[0]);
}

static void iq_mat_mult(const _iq *A, const _iq *B, _iq *C, int rowsA, int colsA, int colsB)
{
    for (int i = 0; i < rowsA; ++i) {
        for (int j = 0; j < colsB; ++j) {
            _iq sum = 0;
            for (int k = 0; k < colsA; ++k) {
                sum += _IQmpy(A[i * colsA + k], B[k * colsB + j]);
            }
            C[i * colsB + j] = sum;
        }
    }
}

static void iq_mat_trans(const _iq *A, _iq *C, int rows, int cols)
{
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            C[j * rows + i] = A[i * cols + j];
        }
    }
}

/* =========================================================================
 *      Kinematics
 * ========================================================================= */

void dynamics_update_trig(float roll, float pitch, float yaw)
{
    s_roll       = _IQ(roll);
    s_pitch      = _IQ(pitch);
    s_yaw        = _IQ(yaw);

    s_sin_roll   = _IQ(arm_sin_f32(roll));
    s_cos_roll   = _IQ(arm_cos_f32(roll));
    s_sin_pitch  = _IQ(arm_sin_f32(pitch));
    s_cos_pitch  = _IQ(arm_cos_f32(pitch));
    s_sin_yaw    = _IQ(arm_sin_f32(yaw));
    s_cos_yaw    = _IQ(arm_cos_f32(yaw));
}

static void dynamics_compute_lambda_iq(_iq out[9])
{
    out[0] = _IQ(1.0f); out[1] = _IQ(0.0f);    out[2] = -s_sin_pitch;
    out[3] = _IQ(0.0f); out[4] =  s_cos_roll;  out[5] =  _IQmpy(s_sin_roll, s_cos_pitch);
    out[6] = _IQ(0.0f); out[7] = -s_sin_roll;  out[8] =  _IQmpy(s_cos_roll, s_cos_pitch);
}

void dynamics_compute_lambda(float out[9])
{
    _iq out_iq[9];
    dynamics_compute_lambda_iq(out_iq);
    for (int i = 0; i < 9; ++i) {
        out[i] = _IQtoF(out_iq[i]);
    }
}

static void dynamics_compute_dlambda_iq(_iq droll, _iq dpitch, _iq out[9])
{
    out[0] = _IQ(0.0f);
    out[1] = _IQ(0.0f);
    out[2] = -_IQmpy(dpitch, s_cos_pitch);

    out[3] = _IQ(0.0f);
    out[4] = -_IQmpy(droll, s_sin_roll);
    out[5] =  _IQmpy(droll, _IQmpy(s_cos_pitch, s_cos_roll))
           -  _IQmpy(dpitch, _IQmpy(s_sin_pitch, s_sin_roll));

    out[6] = _IQ(0.0f);
    out[7] = -_IQmpy(droll, s_cos_roll);
    out[8] = -_IQmpy(droll,  _IQmpy(s_cos_pitch, s_sin_roll))
           -  _IQmpy(dpitch, _IQmpy(s_cos_roll,  s_sin_pitch));
}

void dynamics_compute_dlambda(float droll, float dpitch, float out[9])
{
    _iq out_iq[9];
    dynamics_compute_dlambda_iq(_IQ(droll), _IQ(dpitch), out_iq);
    for (int i = 0; i < 9; ++i) {
        out[i] = _IQtoF(out_iq[i]);
    }
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

float dynamics_compute_U0(const float states[8], float z_k_d)
{
    _iq z_k  = _IQ(states[0]);
    _iq dz_k = _IQ(states[1]);
    _iq z_k_d_iq = _IQ(z_k_d);

    _iq drone_m    = _IQ(DRONE_M);
    _iq gravity    = _IQ(GRAVITY);
    _iq drone_sz   = _IQ(DRONE_SZ);
    _iq dynamics_t = _IQ(DYNAMICS_T);

    _iq t_over_m = _IQdiv(dynamics_t, drone_m);
    _iq m_g = _IQmpy(drone_m, gravity);
    _iq dz_sq = _IQmpy(dz_k, dz_k);
    _iq sz_term = _IQmpy(_IQmpy(drone_sz, iq_sign(dz_k)), dz_sq);
    _iq A = _IQmpy(t_over_m, -m_g - sz_term);

    _iq smc_a_z = _IQ(SMC_A_Z);
    _iq s_z = _IQmpy(smc_a_z, z_k_d_iq - z_k) - dz_k;
    
    _iq sat_s = iq_sat(s_z, _IQ(SMC_EPSILON_Z));

    _iq smc_xi_z = _IQ(SMC_XI_Z);
    _iq smc_e_z  = _IQ(SMC_E_Z);

    _iq term1 = -_IQmpy(_IQmpy(smc_a_z, dynamics_t), dz_k);
    _iq term3 = _IQmpy(_IQmpy(smc_xi_z, dynamics_t), s_z);
    _iq term4 = _IQmpy(_IQmpy(smc_e_z, dynamics_t), sat_s);

    _iq numer = term1 - A + term3 + term4;

    _iq denom = _IQmpy(_IQmpy(s_cos_pitch, s_cos_roll), dynamics_t);
    
    if (_IQabs(denom) < _IQ(DYNAMICS_EPS_DENOM)) {
        return 0.0f;
    }
    
    _iq m_over_denom = _IQdiv(drone_m, denom);
    _iq U0_iq = _IQmpy(m_over_denom, numer);
    
    return _IQtoF(U0_iq);
}

/* =========================================================================
 *      SMC #2 — Attitude controller (outputs U1, U2, U3)
 *
 *   eta   = [roll;  pitch;  yaw]      = states[2..4]
 *   deta  = [droll; dpitch; dyaw]     = states[5..7]
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

void dynamics_compute_attitude_control(const float states[8],
                                       const float eta_k_d[3],
                                       const float w[4],
                                       float U[3])
{
    _iq eta_k[3]  = { _IQ(states[2]), _IQ(states[3]), _IQ(states[4]) };
    _iq deta_k[3] = { _IQ(states[5]), _IQ(states[6]), _IQ(states[7]) };
    _iq eta_k_d_iq[3] = { _IQ(eta_k_d[0]), _IQ(eta_k_d[1]), _IQ(eta_k_d[2]) };
    _iq w_iq[4]       = { _IQ(w[0]), _IQ(w[1]), _IQ(w[2]), _IQ(w[3]) };

    _iq Lambda[9];
    _iq dLambda[9];
    dynamics_compute_lambda_iq(Lambda);
    dynamics_compute_dlambda_iq(deta_k[0], deta_k[1], dLambda);

    _iq I_mat[9];
    for (int i = 0; i < 9; ++i) {
        I_mat[i] = _IQ(DRONE_J_DATA[i]);
    }

    _iq Lambda_T[9], Tmp[9], J_mat[9], J_Lambda[9];

    iq_mat_trans(Lambda, Lambda_T, 3, 3);
    iq_mat_mult(Lambda_T, I_mat, Tmp, 3, 3, 3);
    iq_mat_mult(Tmp, Lambda, J_mat, 3, 3, 3);
    iq_mat_mult(J_mat, Lambda, J_Lambda, 3, 3, 3);

    _iq s_eta[3], sat_s[3];
    _iq smc_a_eta = _IQ(SMC_A_ETA);
    _iq smc_eps_eta = _IQ(SMC_EPSILON_ETA);
    for (int i = 0; i < 3; ++i) {
        s_eta[i] = _IQmpy(smc_a_eta, eta_k_d_iq[i] - eta_k[i]) - deta_k[i];
        sat_s[i] = iq_sat(s_eta[i], smc_eps_eta);
    }

    _iq v_dL[3], term1[3];
    iq_mat_mult(dLambda, deta_k, v_dL, 3, 3, 1);
    iq_mat_mult(J_mat, v_dL, term1, 3, 3, 1);

    _iq Omega[3], JOmega[3];
    iq_mat_mult(Lambda, deta_k, Omega, 3, 3, 1);
    iq_mat_mult(J_mat, Omega, JOmega, 3, 3, 1);

    _iq w_sum = w_iq[0] + w_iq[1] + w_iq[2] + w_iq[3];
    _iq alpha[3] = { 0, 0, _IQmpy(_IQ(CM), w_sum) };

    _iq term2[3], term3[3];
    iq_cross3(Omega, JOmega, term2);
    iq_cross3(Omega, alpha,  term3);

    _iq V[3];
    _iq smc_e_eta = _IQ(SMC_E_ETA);
    _iq smc_xi_eta = _IQ(SMC_XI_ETA);
    _iq dynamics_t = _IQ(DYNAMICS_T);

    for (int i = 0; i < 3; ++i) {
        V[i] = -_IQmpy(_IQmpy(smc_a_eta, dynamics_t), deta_k[i])
             +  _IQmpy(_IQmpy(smc_e_eta, dynamics_t), sat_s[i])
             +  _IQmpy(_IQmpy(smc_xi_eta, dynamics_t), s_eta[i]);
    }

    _iq JL_V[3];
    iq_mat_mult(J_Lambda, V, JL_V, 3, 3, 1);

    _iq inv_T = _IQdiv(_IQ(1.0f), dynamics_t);
    for (int i = 0; i < 3; ++i) {
        _iq U_iq = _IQmpy(JL_V[i], inv_T) + term1[i] + term2[i] + term3[i];
        U[i] = _IQtoF(U_iq);
    }
}




/* =========================================================================
 *      Control allocation — map [U0, U1, U2, U3] -> motor angular vels
 *
 *   [w0^2; w1^2; w2^2; w3^2] = invMot * [U0; U1; U2; U3]
 *   w_i = sqrt(max(w_i^2, 0))
 *
 *   Done in plain float on purpose: invMot entries are O(1e5..1e6) and the
 *   inputs are O(1..10), so the intermediate products are O(1e6..1e7).
 *   In IQ24 those products overflow int64 (need a*b*2^48 < 2^63, i.e.
 *   a*b < 32768), which is why this stage produced garbage.
 *   The FPU on the M33 handles this in a handful of cycles per tick.
 * ========================================================================= */
void dynamics_compute_dw(float w[4], float U[4])
{
    static const float invMot[4][4] = {
        {  99100.0f,   1063900.0f,  -783200.0f,  -336700.0f },
        {  99100.0f,  -1063900.0f,   783200.0f,  -336700.0f },
        {  99100.0f,   1063900.0f,   783200.0f,   336700.0f },
        {  99100.0f,  -1063900.0f,  -783200.0f,   336700.0f }
    };

    for (int i = 0; i < 4; ++i) {
        float w_sq = invMot[i][0] * U[0]
                   + invMot[i][1] * U[1]
                   + invMot[i][2] * U[2]
                   + invMot[i][3] * U[3];

        /* Negative w^2 means the commanded mix is infeasible for this motor;
         * clamp to zero rather than producing a NaN out of sqrtf. */
        if (w_sq < 0.0f) {
            w_sq = 0.0f;
        }
        w[i] = sqrtf(w_sq);
    }
}
