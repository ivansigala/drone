/*
    kalman_z.h
    Author: Diego
    Created on: 21, April 2026
    Updated   : 18, May 2026  — measurement source switched from BME280
                                 hypsometric altitude to a fixed-base
                                 VL53L0X (VLX/ESP32 bridge) that already
                                 reports absolute world-Z position [m].

    Vertical position (z), velocity (ż) and accel-bias Kalman filter —
    pure float32 implementation.

    ┌─────────────────────────────────────────────────────────────────┐
    │  Signal pipeline                                                │
    │                                                                 │
    │  VLX      uint16_t mm ──► float metres ──► direct z measurement │
    │  BNO085   float q     ──► quaternion rotation ──► a_z_world     │
    │                                                                 │
    │  Kalman filter (float32, CMSIS arm_mat_*_f32)                   │
    │    State  x = [altitude_m, velocity_m_s, accel_bias_m_s2] (3×1) │
    │    Input  u = a_z_world  (scalar, from body→world projection)   │
    │    Meas.  y = z_meas_m   (scalar, from VLX bridge)              │
    │                                                                 │
    │  Dynamics:                                                      │
    │    h_k+1 = h_k + dt·v_k + ½dt²·(u_k − b_k)                      │
    │    v_k+1 = v_k + dt·(u_k − b_k)                                 │
    │    b_k+1 = b_k + w_b          (random-walk bias)                │
    │                                                                 │
    │  Covariance update: Joseph form for numerical symmetry          │
    │    P = (I−KH)·P·(I−KH)ᵀ + K·R·Kᵀ                                │
    └─────────────────────────────────────────────────────────────────┘
 */

#ifndef KALMAN_Z_H_
#define KALMAN_Z_H_

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"   /* float32_t, arm_matrix_instance_f32 */


/* =======================================================================
 * Initialisation
 * ===================================================================== */

/*!
 * @brief  Initialise the Kalman filter.
 *
 * Must be called once before any predict / update calls.
 * The filter starts with state x = [0, 0, 0] and large initial covariance.
 *
 * @param Q         Process noise covariance — 3×3 row-major array (9 floats).
 *                  Recommended decomposition  Q = Q_accel + Q_bias :
 *
 *                    Q_accel = σ_az² · [[dt⁴/4, dt³/2, 0],
 *                                        [dt³/2, dt²,   0],
 *                                        [0,     0,     0]]
 *
 *                    Q_bias  =          [[0, 0, 0],
 *                                        [0, 0, 0],
 *                                        [0, 0, q_b]]
 *
 *                  σ_az² comes from accel_covariance.m; q_b is the per-step
 *                  bias random-walk variance — extracted by
 *                  accel_bias_allan.m, or start with q_b ≈ 1e-8 and tune.
 *
 * @param r_z       Measurement noise variance for the VLX Z reading [m²].
 *                  VL53L0X typical 1σ ≈ 0.03 m → start with r_z ≈ 1e-3.
 * @param dt_s      Fixed predict time step [s] — e.g. 0.01 for 100 Hz.
 */
void kalman_z_init(const float32_t Q[9],
                   float32_t r_z,
                   float32_t dt_s);


/* =======================================================================
 * Sensor feed  —  call from your SH2 / baro callbacks
 * ===================================================================== */

/*!
 * @brief  Store the latest BNO085 rotation vector (quaternion).
 *
 * @param qi    Quaternion i component  (sh2_RotationVectorWAcc_t::i)
 * @param qj    Quaternion j component  (sh2_RotationVectorWAcc_t::j)
 * @param qk    Quaternion k component  (sh2_RotationVectorWAcc_t::k)
 * @param qreal Quaternion real / w     (sh2_RotationVectorWAcc_t::real)
 */
void kalman_z_set_attitude(float32_t qi, float32_t qj,
                            float32_t qk, float32_t qreal);

/*!
 * @brief  Run one Kalman predict step using BNO085 linear acceleration.
 *
 * Body-frame acceleration is rotated into the world frame using the stored
 * quaternion; only the world-Z component drives the filter, and the filter's
 * bias estimate is subtracted from it before integration.
 *
 * @param ax  Linear acceleration X [m/s²]
 * @param ay  Linear acceleration Y [m/s²]
 * @param az  Linear acceleration Z [m/s²]
 */
void kalman_z_predict(float32_t ax, float32_t ay, float32_t az);

/*!
 * @brief  Run one Kalman update step from a new VLX Z-position reading.
 *
 *         The VLX is mounted on a fixed base oriented along world-Z, so the
 *         range it reports is the absolute height of the drone — no
 *         coordinate-frame projection or pressure-to-altitude conversion is
 *         required.  Caller is responsible for converting the on-wire
 *         millimetre value to metres.
 *
 * @param z_meas_m  Measured Z position [m].
 */
void kalman_z_update(float32_t z_meas_m);


/* =======================================================================
 * Outputs
 * ===================================================================== */

/*!
 * @brief  Estimated altitude [m], relative to the reference pressure.
 */
float32_t kalman_z_get_altitude(void);

/*!
 * @brief  Estimated vertical velocity [m/s]. Positive = ascending.
 */
float32_t kalman_z_get_velocity(void);

/*!
 * @brief  Estimated world-Z accel bias [m/s²]. Positive = filter sees a
 *         persistent +Z offset in world-frame linear acceleration; this is
 *         what a residual tilt or imperfect gravity removal looks like.
 */
float32_t kalman_z_get_accel_bias(void);

/*!
 * @brief  Z measurement from the last kalman_z_update() [m], expressed
 *         *relative to the reference* — i.e. the value the filter actually
 *         innovated against.  Use this column when fitting the measurement
 *         covariance off-line.
 */
float32_t kalman_z_get_meas_z(void);

/*!
 * @brief  Reference altitude latched on the first kalman_z_update() call
 *         [m, in the same units the caller passes in — VLX metres].
 *         Returns 0 until that first call lands.
 */
float32_t kalman_z_get_reference(void);

/*!
 * @brief  True once the first kalman_z_update() has captured the reference,
 *         false otherwise.  Lets callers tell the difference between
 *         "filter still cold-starting" and "h ≈ 0 because we're at the
 *         reference".
 */
bool kalman_z_reference_set(void);

/*!
 * @brief  Forget the latched reference so the next kalman_z_update() call
 *         re-captures it.  Also re-zeroes the altitude/velocity state
 *         (the accel-bias estimate is preserved).
 */
void kalman_z_reset_reference(void);


#endif /* KALMAN_Z_H_ */
