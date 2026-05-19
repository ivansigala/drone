/*
    kalman_z.c
    Author: Diego
    Created on: 21, April 2026
    Updated   : 18, May 2026  — measurement source switched from BME280
                                 hypsometric altitude to a fixed-base VLX
                                 (VL53L0X / ESP32 bridge) reporting Z in mm.

    3-state vertical Kalman filter — pure float32 implementation.

    ╔══════════════════════════════════════════════════════════════════╗
    ║  State:  x = [h, v, b]ᵀ                                          ║
    ║     h  altitude                       [m]                        ║
    ║     v  vertical velocity              [m/s]                      ║
    ║     b  world-Z accel bias             [m/s²]                     ║
    ║                                                                  ║
    ║  Input: u = a_z_world  (quaternion-rotated SH2 linear accel)     ║
    ║  Meas.: y = z_meas_m   (absolute world-Z from the VLX bridge)    ║
    ║                                                                  ║
    ║  Dynamics  (discrete, timestep dt):                              ║
    ║     h_k+1 = h_k + dt·v_k + ½dt²·(u_k − b_k)                      ║
    ║     v_k+1 = v_k + dt·(u_k − b_k)                                 ║
    ║     b_k+1 = b_k + w_b          (random-walk)                     ║
    ║                                                                  ║
    ║  ⇒  F = [[1, dt, −½dt²],                                         ║
    ║          [0,  1,   −dt ],                                        ║
    ║          [0,  0,    1  ]]                                        ║
    ║                                                                  ║
    ║      B = [½dt²; dt; 0]   (applied to u = a_z_world)              ║
    ║                                                                  ║
    ║      H = [1, 0, 0]       (VLX observes altitude only)            ║
    ║                                                                  ║
    ║  Covariance update: Joseph form for numerical symmetry           ║
    ║    P = (I−KH)·P·(I−KH)ᵀ + K·R·Kᵀ                                 ║
    ╚══════════════════════════════════════════════════════════════════╝

    World-frame rotation  (BNO085 quaternion: body → world)
    ────────────────────────────────────────────────────────
      q = [qw, qx, qy, qz]          (unit quaternion)
      a_world_z = 2(qx·qz − qw·qy)·ax
                + 2(qy·qz + qw·qx)·ay
                + (1 − 2(qx² + qy²))·az
 */

#include "kalman_z.h"
#include <string.h>     /* memcpy, memset */
#include "arm_math.h"

/* -----------------------------------------------------------------------
 * Module constants
 * --------------------------------------------------------------------- */
#define KZ_N            3           /* state dimension                              */

/* -----------------------------------------------------------------------
 * Internal state  (pure float — no fixed-point fields)
 * --------------------------------------------------------------------- */
typedef struct {

    /* ── Kalman float state ───────────────────────────────────────── */
    float32_t  x[3];      /* state vector   [h_m, v_m_s, b_m_s2]        */
    float32_t  P[9];      /* 3×3 covariance  (row-major)                 */
    float32_t  Q[9];      /* 3×3 process noise (full matrix)             */
    float32_t  F[9];      /* 3×3 state transition                        */
    float32_t  R;         /* VLX measurement noise variance              */
    float32_t  dt;        /* fixed predict time step [s]                 */

    /* ── CMSIS-DSP matrix instances (pData → arrays above) ────────── */
    arm_matrix_instance_f32 F_mat;
    arm_matrix_instance_f32 P_mat;
    arm_matrix_instance_f32 Q_mat;

    /* ── Attitude in float:  [qw, qx, qy, qz]  (body→world) ───────── */
    float32_t  q[4];

    /* ── Raw VLX Z measurement from last update() — diagnostics ─────
     *  Stored *after* the reference subtraction so it matches the
     *  innovation actually fed to the filter.                          */
    float32_t  meas_z_m;

    /* ── Reference altitude (auto-captured on the first update) ─────
     *  ref_set goes true the first time kalman_z_update() runs and
     *  stays true until kalman_z_reset_reference() is called.  Once
     *  set, every incoming measurement is offset:
     *      z_rel = z_meas − z_ref
     *  so the filter's state is always height above the reference.    */
    float32_t  z_ref_m;
    bool       ref_set;

} kalman_z_state_t;

static kalman_z_state_t s_kz;

/* =======================================================================
 * Public API
 * ===================================================================== */

void kalman_z_init(const float32_t Q[9],
                   float32_t r_z,
                   float32_t dt_s)
{
    memset(&s_kz, 0, sizeof(s_kz));

    s_kz.dt = dt_s;
    s_kz.R  = r_z;

    /* ── State transition ─────────────────────────────────────────────
     *        [1,  dt,  −½dt²]
     *   F =  [0,   1,   −dt ]
     *        [0,   0,    1  ]                                         */
    const float32_t dt        = dt_s;
    const float32_t half_dt2  = 0.5f * dt * dt;
    s_kz.F[0] = 1.0f;  s_kz.F[1] = dt;    s_kz.F[2] = -half_dt2;
    s_kz.F[3] = 0.0f;  s_kz.F[4] = 1.0f;  s_kz.F[5] = -dt;
    s_kz.F[6] = 0.0f;  s_kz.F[7] = 0.0f;  s_kz.F[8] = 1.0f;

    /* ── Process noise — full 3×3 (copy caller's matrix) ───────────── */
    memcpy(s_kz.Q, Q, 9 * sizeof(float32_t));

    /* ── Initial covariance  P₀  — diagonal, large uncertainty ───────
     *   Altitude/velocity: 1000 (m² and (m/s)²)
     *   Bias            :  1    (m/s²)² ≈ ±1 m/s² 1σ — generous       */
    memset(s_kz.P, 0, sizeof(s_kz.P));
    s_kz.P[0] = 1000.0f;
    s_kz.P[4] = 1000.0f;
    s_kz.P[8] =    1.0f;

    /* ── Bind CMSIS matrix instances to static data arrays ──────────── */
    arm_mat_init_f32(&s_kz.F_mat, KZ_N, KZ_N, s_kz.F);
    arm_mat_init_f32(&s_kz.P_mat, KZ_N, KZ_N, s_kz.P);
    arm_mat_init_f32(&s_kz.Q_mat, KZ_N, KZ_N, s_kz.Q);

    /* ── Identity quaternion  q = [1, 0, 0, 0] ─────────────────────── */
    s_kz.q[0] = 1.0f;   /* qw */
    s_kz.q[1] = 0.0f;   /* qx */
    s_kz.q[2] = 0.0f;   /* qy */
    s_kz.q[3] = 0.0f;   /* qz */

    s_kz.meas_z_m = 0.0f;
    s_kz.z_ref_m  = 0.0f;
    s_kz.ref_set  = false;
}

/* ----------------------------------------------------------------------- */

void kalman_z_set_attitude(float32_t qi, float32_t qj,
                            float32_t qk, float32_t qreal)
{
    /* Internal order [w, x, y, z] — matches body→world rotation convention */
    s_kz.q[0] = qreal;
    s_kz.q[1] = qi;
    s_kz.q[2] = qj;
    s_kz.q[3] = qk;
}

/* ----------------------------------------------------------------------- */

void kalman_z_predict(float32_t ax, float32_t ay, float32_t az)
{
    /* ── Step 1: Project body-frame accel onto world vertical axis ────
     *
     *  BNO085 rotation vector q = [qw, qx, qy, qz] rotates vectors from
     *  the body frame into the world (Earth) frame.  The third row of the
     *  resulting rotation matrix dotted with (ax, ay, az) gives a_z_world:
     *
     *    R31 =   2(qx·qz − qw·qy)
     *    R32 =   2(qy·qz + qw·qx)
     *    R33 =   1 − 2(qx² + qy²)                                       */
    const float32_t qw = s_kz.q[0];
    const float32_t qx = s_kz.q[1];
    const float32_t qy = s_kz.q[2];
    const float32_t qz = s_kz.q[3];

    const float32_t a_z =  2.0f * (qx*qz - qw*qy) * ax
                        +  2.0f * (qy*qz + qw*qx) * ay
                        + (1.0f - 2.0f * (qx*qx + qy*qy)) * az;

    /* ── Step 2: State predict  x̂ = F·x̂ + B·a_z
     *   With bias subtracted, the kinematics are:
     *       h += dt·v + ½dt²·(u − b)
     *       v += dt·(u − b)
     *       b  unchanged                                                 */
    const float32_t dt       = s_kz.dt;
    const float32_t half_dt2 = 0.5f * dt * dt;
    const float32_t a_corr   = a_z - s_kz.x[2];            /* u − b       */

    s_kz.x[0] += dt * s_kz.x[1] + half_dt2 * a_corr;       /* altitude    */
    s_kz.x[1] += dt * a_corr;                              /* velocity    */
    /* s_kz.x[2] unchanged on predict (random-walk mean)                  */

    /* ── Step 3: Covariance predict  P = F·P·Fᵀ + Q  (CMSIS-DSP, 3×3) ── */
    float32_t FP_buf[9], Ft_buf[9], FPFt_buf[9], Ppred_buf[9];
    arm_matrix_instance_f32 FP_mat, Ft_mat, FPFt_mat, Ppred_mat;

    arm_mat_init_f32(&FP_mat,    KZ_N, KZ_N, FP_buf);
    arm_mat_init_f32(&Ft_mat,    KZ_N, KZ_N, Ft_buf);
    arm_mat_init_f32(&FPFt_mat,  KZ_N, KZ_N, FPFt_buf);
    arm_mat_init_f32(&Ppred_mat, KZ_N, KZ_N, Ppred_buf);

    arm_mat_mult_f32 (&s_kz.F_mat, &s_kz.P_mat, &FP_mat);   /* F · P       */
    arm_mat_trans_f32(&s_kz.F_mat, &Ft_mat);                /* Fᵀ          */
    arm_mat_mult_f32 (&FP_mat, &Ft_mat, &FPFt_mat);         /* F · P · Fᵀ */
    arm_mat_add_f32  (&FPFt_mat, &s_kz.Q_mat, &Ppred_mat);  /* + Q         */

    arm_copy_f32(Ppred_buf, s_kz.P, 9);
}

/* ----------------------------------------------------------------------- */

void kalman_z_update(float32_t z_meas_m)
{
    /* ── Step 0: First-call reference capture ───────────────────────
     *   Latch the very first measurement as the zero reference so the
     *   filter's altitude state always represents height *above* the
     *   spot where logging started — the kinematics and covariance
     *   tuning are easier to interpret when h₀ = 0.                    */
    if (!s_kz.ref_set) {
        s_kz.z_ref_m = z_meas_m;
        s_kz.ref_set = true;
    }

    /* Convert the raw measurement to a value relative to the reference,
     * then cache it for diagnostics — this is exactly what the filter
     * is innovating against.                                            */
    const float32_t z_rel_m = z_meas_m - s_kz.z_ref_m;
    s_kz.meas_z_m = z_rel_m;

    /* ── Step 2: Innovation covariance  S = H·P·Hᵀ + R ─────────────
     *   H = [1, 0, 0]  →  H·P·Hᵀ = P[0][0]                              */
    float32_t S = s_kz.P[0] + s_kz.R;
    if (S < 1.0e-9f) S = 1.0e-9f;

    /* ── Step 3: Kalman gain  K = P·Hᵀ / S   (3×1 column) ──────────── */
    float32_t K0 = s_kz.P[0] / S;   /* P[0][0]  — alt / alt             */
    float32_t K1 = s_kz.P[3] / S;   /* P[1][0]  — vel / alt             */
    float32_t K2 = s_kz.P[6] / S;   /* P[2][0]  — bias / alt            */

    /* ── Step 4: Innovation  ν = z_rel − H·x̂ ──────────────────────── */
    float32_t innov = z_rel_m - s_kz.x[0];

    /* ── Step 5: State update  x̂ += K·ν ──────────────────────────── */
    s_kz.x[0] += K0 * innov;
    s_kz.x[1] += K1 * innov;
    s_kz.x[2] += K2 * innov;

    /* ── Step 6: Covariance update — Joseph form ─────────────────────
     *   P = (I − K·H) · P · (I − K·H)ᵀ + K·R·Kᵀ
     *   With H = [1, 0, 0]:
     *                 [1−K0, 0, 0]
     *       I − K·H = [−K1,  1, 0]
     *                 [−K2,  0, 1]                                     */
    float32_t IKH_buf[9] = {
        1.0f - K0,  0.0f,  0.0f,
        -K1,        1.0f,  0.0f,
        -K2,        0.0f,  1.0f
    };
    float32_t IKHt_buf[9], temp_buf[9], Pnew_buf[9];
    arm_matrix_instance_f32 IKH_mat, IKHt_mat, temp_mat, Pnew_mat;

    arm_mat_init_f32(&IKH_mat,  KZ_N, KZ_N, IKH_buf);
    arm_mat_init_f32(&IKHt_mat, KZ_N, KZ_N, IKHt_buf);
    arm_mat_init_f32(&temp_mat, KZ_N, KZ_N, temp_buf);
    arm_mat_init_f32(&Pnew_mat, KZ_N, KZ_N, Pnew_buf);

    arm_mat_trans_f32(&IKH_mat, &IKHt_mat);              /* (I−KH)ᵀ       */
    arm_mat_mult_f32 (&IKH_mat, &s_kz.P_mat, &temp_mat); /* (I−KH)·P      */
    arm_mat_mult_f32 (&temp_mat, &IKHt_mat, &Pnew_mat);  /*     · (I−KH)ᵀ */

    /* + K·R·Kᵀ  (outer product, 3×3, element-wise)                       */
    const float32_t rK0 = s_kz.R * K0;
    const float32_t rK1 = s_kz.R * K1;
    const float32_t rK2 = s_kz.R * K2;
    Pnew_buf[0] += rK0 * K0;  Pnew_buf[1] += rK0 * K1;  Pnew_buf[2] += rK0 * K2;
    Pnew_buf[3] += rK1 * K0;  Pnew_buf[4] += rK1 * K1;  Pnew_buf[5] += rK1 * K2;
    Pnew_buf[6] += rK2 * K0;  Pnew_buf[7] += rK2 * K1;  Pnew_buf[8] += rK2 * K2;

    arm_copy_f32(Pnew_buf, s_kz.P, 9);
}

/* =======================================================================
 * Output getters  —  pure float32
 * ===================================================================== */

float32_t kalman_z_get_altitude(void)   { return s_kz.x[0];       }
float32_t kalman_z_get_velocity(void)   { return s_kz.x[1];       }
float32_t kalman_z_get_accel_bias(void) { return s_kz.x[2];       }
float32_t kalman_z_get_meas_z(void)     { return s_kz.meas_z_m;   }

float32_t kalman_z_get_reference(void)  { return s_kz.z_ref_m;    }
bool      kalman_z_reference_set(void)  { return s_kz.ref_set;    }

/* -----------------------------------------------------------------------
 *  kalman_z_reset_reference
 *
 *  Clear the latched reference altitude so the *next* kalman_z_update()
 *  call re-captures it.  Useful after picking the drone up, repositioning
 *  the base, or otherwise wanting to re-zero the filter without doing a
 *  full kalman_z_init() (which would also wipe Q/R and covariance).
 *  The current state estimate is also re-zeroed so the filter doesn't
 *  inherit a stale altitude that no longer matches the new reference.
 * --------------------------------------------------------------------- */
void kalman_z_reset_reference(void)
{
    s_kz.ref_set  = false;
    s_kz.z_ref_m  = 0.0f;
    s_kz.x[0]     = 0.0f;
    s_kz.x[1]     = 0.0f;
    /* leave x[2] (accel bias) alone — it's still valid                  */
}
