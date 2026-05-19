/*
 * Author: Diego 
 * Created on: 6, April 2026
 */

#include <math.h>
#include <string.h>     /* memcpy for the debug-UART payload pack */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"

/* user includes. */
#include "kalman_z.h"
#include "bno_08x.h"
#include "vlx_esp32.h"
#include "rc_fsia6B.h"
#include "dshot.h"
#include "pid.h"
#include "dynamics.h"
#include "euler.h"
#include "timer_driver_mcxn947.h"
#include "debug_uart_mcxn947.h"
#include "gpio_driver_mcxn947.h"

/* Macro helper to print negative values */
#define SIGN_STR(x)        ((x) < 0.0f ? "-" : "")
#define ABS_F(x)           ((x) < 0.0f ? -(x) : (x))

/* Baud rate for the non-blocking debug stream (LPUART4 -> USB-CDC -> MATLAB). */
#define DEBUG_UART_BAUDRATE 115200U
#define PID_FREQUENCY_HZ 100.0f
#define CONTROL_LOOP_PERIOD_S (1.0f / PID_FREQUENCY_HZ)

/*
 * Maximum mechanical angular velocity the motors are expected to reach at
 * full DSHOT throttle.  Adjust to match your specific motor + propeller.
 *   erpm_max ≈ DSHOT_MAX_THROTTLE * scale_factor (ESC-dependent)
 *   omega_max = erpm_max * 100 / DSHOT_MOTOR_POLE_PAIRS * (2π / 60)
 * 3000 rad/s ≈ 28 650 RPM on a 7-pole-pair motor — a reasonable upper bound
 * for typical 2204-class outrunners.
 */
#define MAX_MOTOR_OMEGA_RAD_S   3000.0f

/*
 * Fraction of the RC throttle range below which all motors are commanded
 * off (throttle_u16 = 0).  Prevents the drone from creeping at minimum
 * throttle when the stick is at the bottom.
 */
#define THROTTLE_DEADBAND_NORM  0.05f

/*
 * PID gains — PLACEHOLDER values only.
 * DO NOT fly without bench-tuning these first (e.g. via Ziegler-Nichols or
 * step-response fitting in MATLAB with the existing telemetry stream).
 *
 *   Kp  : primary gain; increase until the motor tracks setpoint changes
 *          quickly without oscillation.
 *   Ki  : integrates away steady-state speed error; keep small to start.
 *   Kd  : damps oscillations; the library uses derivative-on-measurement so
 *          it won't spike on a step setpoint change.
 *
 * i_min / i_max clamp the integrator to ±30 % of the output range so that
 * windup during spin-up or blocked-prop events stays bounded.
 */
#define PID_KP          0.5f
#define PID_KI          0.0f
#define PID_KD          0.0f
#define PID_I_MIN      -100.0f
#define PID_I_MAX       100.0f

/*
 * Error deadband for the speed PID (rad/s).
 *
 * The ESC telemetry reports eRPM in steps of 100 eRPM.  Converting to
 * mechanical rad/s:
 *
 *   Δω = 100 eRPM × (100 / pole_pairs) × (2π / 60)
 *      = 100 × (100 / 7) × (2π / 60)  ≈ 10.5 rad/s   (for 7 pole-pairs)
 *
 * Any speed error smaller than this cannot be distinguished from a
 * quantisation step in the sensor, so the PID treats it as zero:
 * the integral stops accumulating and the P term is suppressed,
 * eliminating steady hunting around the setpoint.
 */
#define PID_ERROR_DEADBAND_RAD_S  10.5f

/*************************************************************************
 * Kalman-Z filter config
 *************************************************************************/

/*
 * Below this target speed, the control loop short-circuits to "motor
 * off" -- it sends DSHOT throttle = 0 (true off, motor freewheels) and
 * resets the PID state to keep the integrator from winding up. The
 * threshold is set above PID_ERROR_DEADBAND_RAD_S so that the deadband
 * around a non-zero setpoint never accidentally trips the off-case.
 */
#define TARGET_OFF_THRESHOLD_RAD_S  20.0f

/*
 * RC channel 7 acts as a hard motor-enable switch.
 *
 *   CH7 ≈ 1000 us  →  motors are FORCED OFF (s_motor_cmd[i] = 0). The
 *                     PID state is flushed every tick while the switch
 *                     is in this position so the integral error cannot
 *                     accumulate against the (always-zero) error and
 *                     spike the moment the pilot flips back to 2000.
 *
 *   CH7 ≈ 2000 us  →  motors are ARMED; normal speed-PID control runs.
 *
 * A single midpoint threshold (1500 us) is used. The FS-i6 transmitter
 * snaps CH7 between 1000 and 2000 with a two-position switch, so the
 * channel value never sits near the threshold for long -- no hysteresis
 * is needed.
 */
#define RC_CH7_ENABLE_THRESHOLD_US  1500U

/*
 * Tilt failsafe — uses the BNO085 rotation vector to detect when the
 * drone has rolled/pitched beyond a safe angle and forces motors off
 * through the same disarm code path as CH7.
 *
 *   tilt_angle = acos(1 - 2*(qi² + qj²))
 *
 * where (qreal, qi, qj, qk) is the body→world quaternion delivered by
 * the BNO085 rotation-vector report. The (3,3) element of the
 * corresponding rotation matrix is (1 - 2(qi² + qj²)) = cos(tilt), the
 * cosine of the angle between body-Z and world-Z. Comparing against a
 * pre-computed cosf(10°) avoids an acosf() in the hot path.
 *
 * The failsafe LATCHES: once tripped, motors stay disarmed until the
 * pilot flips CH7 to the disarm position (which clears the latch) and
 * then re-arms. This prevents motor-cmd chatter from a tilt that
 * hovers around the threshold and forces a deliberate pilot ack after
 * a crash event.
 */
#define TILT_FAILSAFE_THRESHOLD_DEG   10.0f
#define TILT_FAILSAFE_COS_THRESHOLD   0.93969262078f   /* cosf(10° * π/180) */

/*
 * Pilot reference scaling for the outer SMC loop.
 *
 *   CH1 (roll  reference) :  [1000, 2000] us  →  [-10°, +10°]   in rad
 *   CH2 (Z     reference) :  [1000, 2000] us  →  [   0,  0.2]  m
 *   CH3 (pitch reference) :  [1000, 2000] us  →  [-10°, +10°]   in rad
 *   yaw reference         :  constant 0 rad
 *
 * Stick values outside [1000, 2000] us are clamped to the corresponding
 * reference limits so a glitched packet cannot command an extreme
 * attitude or altitude.
 *
 * Z range is intentionally tight (20 cm above take-off) — the VL53L0X
 * is most accurate in its near-field band, and limiting the commandable
 * altitude keeps the drone within the sensor's high-confidence range
 * during early bench / hover testing.
 */
#define DEG_TO_RAD            0.017453292519943295f
#define ATTITUDE_REF_MAX_RAD  (10.0f * DEG_TO_RAD)
#define Z_REF_MAX_M           0.20f
#define RC_PULSE_MID_US       1500.0f
#define RC_PULSE_HALF_US      500.0f
#define RC_PULSE_MIN_US       1000.0f
#define RC_PULSE_RANGE_US     1000.0f

/*
 * IMU staleness failsafe.
 *
 * The SH2 callback timestamps every rotation-vector and gyroscope
 * update. The control loop forces the disarm path if the IMU has
 * been silent for more than IMU_STALE_TIMEOUT_MS, so a frozen sensor
 * (I²C glitch, crash, power dip) can't keep feeding stale attitude
 * into the SMC. 50 ms is five expected 100 Hz cycles — long enough
 * to absorb normal jitter, short enough to react before a runaway.
 *
 * Unlike the tilt latch, IMU staleness clears automatically the
 * instant a fresh sample arrives. It does NOT clear the tilt latch
 * (only a CH7 disarm does).
 */
#define IMU_STALE_TIMEOUT_MS  50U

#define IMU_NOTIFY_BIT            (1u << 0)

/* -----------------------------------------------------------------------
 *  Kalman tuning  —  VLX (ESP32 / VL53L0X bridge) measurement source.
 *  Constants mirror frdmmcxn947_freertos_imu/main.c exactly.
 *
 *  dt        : SH2 linear-accel reports at 100 Hz, so each predict
 *              advances the filter by 10 ms.
 *  KZ_R_Z    : variance of the VLX Z measurement [m²].  VL53L0X 1σ ≈
 *              1.6 mm after ESP-NOW jitter is factored in.
 *  σ_az²     : variance of (v_k − v_{k−1})/dt from the BNO085 linear-accel
 *              stream; feeds the kinematic block of Q.
 *  q_bias    : Allan-variance slope for the world-Z accel bias —
 *              155 windows of 100 samples (see accel_bias_allan.m).
 *
 *  Q is decomposed as
 *      Q = σ_az² · [[dt⁴/4, dt³/2, 0],
 *                   [dt³/2, dt²,   0],
 *                   [0,     0,     0]]
 *        +         [[0, 0, 0],
 *                   [0, 0, 0],
 *                   [0, 0, q_b]]
 *  and the resulting matrix is pre-computed for dt = 10 ms so the SensorTask
 *  doesn't have to rebuild it at runtime.
 * --------------------------------------------------------------------- */
#define KZ_DT                     0.010f          /* fixed predict period [s] (100 Hz)      */
#define KZ_R_Z                    2.427409e-06f   /* var(z_meas) — VLX 1σ ≈ 0.0016 m         */

static const float32_t KZ_Q[9] = {
    /* row 0: h / {h, v, b}    —   σ_az²·dt⁴/4,  σ_az²·dt³/2,  0      */
    1.054996e-13f,   2.109992e-11f,   0.0f,
    /* row 1: v / {h, v, b}    —   σ_az²·dt³/2,  σ_az²·dt²,    0      */
    2.109992e-11f,   4.219983e-09f,   0.0f,
    /* row 2: b / {h, v, b}    —   0,            0,            q_bias */
    0.0f,            0.0f,            1.144956e-08f
};

/* Debug-UART print rate for the Kalman-Z output. 50 Hz = every 20 ms. */
#define KALMAN_Z_PRINT_PERIOD_MS  20U

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Task priorities. */
#define RC_task_PRIORITY 4
#define ESC_task_PRIORITY 4
#define MOTOR_task_PRIORITY 4
#define CONTROL_task_PRIORITY 4
#define SENSOR_task_PRIORITY 4
#define sensor_task_PRIORITY 2

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void RCParserTask(void *pvParameters);
static void ESCTelemetryTask(void *pvParameters);
static void DSHOTGeneratorTask(void *pvParameters);
static void ControlLoopTask(void *pvParameters);
static void SensorTask(void *pvParameters);

void RC_Callback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData);
void ESC_Callback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData);


/*******************************************************************************
 * Variables
 ******************************************************************************/
AT_NONCACHEABLE_SECTION_INIT(uint8_t g_rc_rxBuffer[FS_IA6B_FRAME_SIZE]) = {0};
AT_NONCACHEABLE_SECTION_INIT(uint8_t g_esc_rxBuffer[DSHOT_TELEMETRY_FRAME_SIZE]) = {0};

/* Sensor handles */
imu_ctrl_t     imu;
vlx_ctrl_t     vlx;            /* ESP32 SPI-slave VLX bridge — replaces BME280 */

/* Task handle for notifications */
TaskHandle_t rcParserTaskHandle      = NULL;
TaskHandle_t escParserTaskHandle     = NULL;
TaskHandle_t dshotGeneratorTaskHandle = NULL;
TaskHandle_t controlLoopTaskHandle   = NULL;
TaskHandle_t   sensorTaskHandle = NULL;

/* Queues */
QueueHandle_t rcChannelQueue  = NULL;
QueueHandle_t escChannelQueue = NULL;

/*
 * Shared state between tasks — written and read by different tasks.
 *
 * s_latest_rc      : most-recent validated RC channel set.
 *                    Written by RCParserTask, read by ControlLoopTask.
 *                    All fields are 16-bit, so each element write is
 *                    naturally atomic on Cortex-M33 (single-cycle STR).
 *
 * s_motor_cmd      : per-motor DSHOT throttle commanded by the PID loop.
 *                    Written by ControlLoopTask (100 Hz), read by
 *                    DSHOTGeneratorTask (400 Hz).  A one-cycle-stale read
 *                    in the DSHOT task is inconsequential.
 */
static volatile fs_ia6b_channels_t s_latest_rc    = {0};
static volatile uint16_t           s_motor_cmd[MAX_SUPPORTED_MOTORS]   = {0};

/*
 * Per-motor PID setpoint, in rad/s. Written by ControlLoopTask each
 * iteration, read by ESCTelemetryTask when it builds the debug-UART
 * frame so MATLAB can plot the setpoint alongside the measured speed.
 * Float reads/writes are 32-bit aligned, atomic on Cortex-M33.
 */
static volatile float              s_motor_target_omega[MAX_SUPPORTED_MOTORS] = {0};

/*
 * Tilt failsafe latch. Set by sh2_sensor_callback() when the rotation
 * vector reports a tilt angle greater than TILT_FAILSAFE_THRESHOLD_DEG.
 * Cleared by ControlLoopTask only when it observes a CH7 disarm
 * (CH7 < RC_CH7_ENABLE_THRESHOLD_US), so the pilot must consciously
 * cycle the kill switch before the drone can be re-armed.
 *
 * Single-byte writes/reads are naturally atomic on Cortex-M33 so no
 * mutex is needed between the SH2 callback and the control loop.
 */
static volatile bool s_tilt_failsafe_latched = false;

/*
 * Latest attitude estimate and body-frame angular rates published by
 * the BNO085 SH2 callback, consumed by ControlLoopTask each tick to
 * build the 8-state vector for the SMC dynamics.
 *
 *   s_roll / s_pitch / s_yaw   : ZYX-like Euler angles [rad], derived
 *                                from the rotation-vector quaternion
 *                                inside sh2_sensor_callback. Match the
 *                                dynamics' Lambda convention (see
 *                                dynamics.c: roll-pitch-yaw with yaw
 *                                applied first).
 *   s_omega_x / y / z          : Calibrated gyroscope output [rad/s],
 *                                body-frame. The control loop converts
 *                                these to Euler rates (droll/dpitch/
 *                                dyaw) via Lambda^-1 before handing the
 *                                state vector to the dynamics module.
 *
 * Each float is 32-bit aligned so individual writes/reads are atomic
 * on Cortex-M33; the three angles (and the three rates) can drift one
 * sample out of step relative to each other, but at the 100 Hz IMU /
 * 100 Hz control cadence that is inconsequential.
 */
static volatile float s_roll    = 0.0f;
static volatile float s_pitch   = 0.0f;
static volatile float s_yaw     = 0.0f;
static volatile float s_omega_x = 0.0f;
static volatile float s_omega_y = 0.0f;
static volatile float s_omega_z = 0.0f;

/*
 * Freshness counters for the IMU streams used by ControlLoopTask.
 *
 *   s_imu_last_update_ms : LPTMR0 millisecond timestamp of the most
 *                          recent rotation-vector OR gyroscope sample.
 *                          ControlLoopTask compares this against
 *                          timer_get_ms() to detect a stuck IMU.
 *   s_imu_ever_received  : starts false and flips true on the first
 *                          IMU sample. Avoids treating "no sample
 *                          yet" as "fresh sample at t=0" before the
 *                          BNO085 has come up.
 */
static volatile uint32_t s_imu_last_update_ms = 0U;
static volatile bool     s_imu_ever_received  = false;

/* ESC Handle */
dshotSystem_t esc;

/* Global variables for telemetry data */
volatile uint8_t g_motor_id = 0;
volatile uint8_t g_erpm_low = 0;
volatile uint8_t g_erpm_high = 0;

static volatile uint8_t  s_pending_motor_id   = 0xFF;   /* 0xFF = idle */
static volatile bool     s_response_in_flight = false;
static volatile bool g_kalman_ready = false;

timer_ctrl_t timer_0 ={
    .timer_id = 0,
    .frequency = 400
};


/*******************************************************************************
 * Code
 ******************************************************************************/

void RC_Callback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (kStatus_LPUART_RxIdle == status)
    {
        /* Notify the parsing task that data is ready */
        vTaskNotifyGiveFromISR(rcParserTaskHandle, &xHigherPriorityTaskWoken);
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ESC_Callback(LPUART_Type *base, lpuart_edma_handle_t *handle,
                  status_t status, void *userData)
{
    BaseType_t hpw = pdFALSE;
    /* Hand any non-success status off to the task too -- the task is
       the single owner of the recovery path, the ISR just signals. */
    if (status == kStatus_LPUART_RxIdle ||
        status == kStatus_LPUART_RxHardwareOverrun ||
        status == kStatus_LPUART_FramingError ||
        status == kStatus_LPUART_NoiseError    ||
        status == kStatus_LPUART_ParityError) {
        vTaskNotifyGiveFromISR(escParserTaskHandle, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
}

void timer_0_callback(void *args) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Notify the motor task to send the next DShot command */
    vTaskNotifyGiveFromISR(dshotGeneratorTaskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

void IMU_Update_Callback(void)
{
    gpio_clear_interrupt_flag(imu.gpio_event.gpio_base, imu.gpio_event.pin);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (sensorTaskHandle != NULL)
    {
        xTaskNotifyFromISR(sensorTaskHandle, IMU_NOTIFY_BIT,
                           eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*!
 * @brief SH2 sensor callback — decoded inside the bno_08x driver service
 *        loop.  Drives the Kalman filter:
 *
 *          SH2_ROTATION_VECTOR    → kalman_z_set_attitude (body→world)
 *          SH2_LINEAR_ACCELERATION → kalman_z_predict     (100 Hz)
 *
 *        Both branches are gated behind g_kalman_ready so the filter is
 *        only touched after SensorTask has finished kalman_z_init().
 */
void sh2_sensor_callback(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t sensorValue;

    if (sh2_decodeSensorEvent(&sensorValue, event) != SH2_OK) {
        return;
    }

    /* Never call into the filter before it has been initialised. */
    if (!g_kalman_ready) {
        return;
    }

    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        sh2_RotationVectorWAcc_t *rv = &sensorValue.un.rotationVector;
        /* Push the latest body→world quaternion into the filter so the
         * next predict() can project body-frame accel onto world Z.        */
        kalman_z_set_attitude(rv->i, rv->j, rv->k, rv->real);

        /*
         * Tilt failsafe.
         *
         * The (3,3) element of the rotation matrix built from the
         * body→world quaternion (w, i, j, k) is
         *
         *     R33 = 1 - 2*(i² + j²) = cos(tilt)
         *
         * where 'tilt' is the angle between the drone's body Z-axis
         * and the world Z-axis (i.e. how far off level the drone is,
         * independent of yaw). When that cosine drops below
         * cosf(10°) the drone has tilted past the safe limit and we
         * latch the failsafe; ControlLoopTask picks it up on its
         * next iteration and runs the CH7 disarm path. The latch
         * stays set until the pilot toggles CH7 to disarm.
         */
        float cos_tilt = 1.0f - 2.0f * (rv->i * rv->i + rv->j * rv->j);
        if (cos_tilt < TILT_FAILSAFE_COS_THRESHOLD) {
            s_tilt_failsafe_latched = true;
        }

        /*
         * Euler-angle extraction for the SMC dynamics module.
         *
         * Use the existing q_to_{roll,pitch,yaw} helpers from euler.c
         * (rather than q_to_ypr, whose header/implementation argument
         * order disagree). The three values feed states[2..4] of the
         * 8-state vector and seed dynamics_update_trig() each tick.
         * Units: radians.
         *
         *   IMU mounting note
         *   -----------------
         *   The BNO085 is rotated 90° about the body-Z axis on this
         *   airframe, so the sensor's X-axis aligns with the drone's
         *   Y-axis and vice versa.  As a result, what the quaternion
         *   decomposition calls "roll" (rotation about IMU-X) is
         *   physically the drone's pitch, and the IMU's "pitch" is the
         *   drone's roll.  We swap them here so the SMC sees angles in
         *   the airframe's reference frame.  Yaw (rotation about body-Z)
         *   is unaffected by a yaw-only re-mounting.
         *
         *   If a future log shows the swapped axis with the wrong sign
         *   (e.g. stick-right rolls the drone left), the IMU was rotated
         *   the opposite way around Z — negate the appropriate line.
         */
        s_roll  = q_to_pitch(rv->real, rv->i, rv->j, rv->k);  /* drone roll ← IMU pitch */
        s_pitch = q_to_roll (rv->real, rv->i, rv->j, rv->k);  /* drone pitch ← IMU roll */
        s_yaw   = q_to_yaw  (rv->real, rv->i, rv->j, rv->k);

        /* Freshness stamp for the staleness failsafe. */
        s_imu_last_update_ms = timer_get_ms();
        s_imu_ever_received  = true;

        /* Tag "R" — quaternion components (body→world) used by the filter
         * to rotate body-frame accel into world frame.  Order matches
         * kalman_z_set_attitude: qreal, qi, qj, qk.                        */
        // PRINTF("R: %s%.6f, %s%.6f, %s%.6f, %s%.6f\r\n",
        //        SIGN_STR(rv->real), ABS_F(rv->real),
        //        SIGN_STR(rv->i),    ABS_F(rv->i),
        //        SIGN_STR(rv->j),    ABS_F(rv->j),
        //        SIGN_STR(rv->k),    ABS_F(rv->k));
    }
    else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        /*
         * Body-frame angular velocity from the BNO085 calibrated
         * gyroscope (rad/s). Stored verbatim; ControlLoopTask
         * converts to Euler rates via Lambda^-1 using the latest
         * roll / pitch so the units match what the SMC attitude
         * controller expects in states[5..7].
         *
         * Same IMU mounting note as in the rotation-vector branch:
         * the BNO085 is rotated 90° about body-Z relative to the
         * airframe, so the sensor's ωx is rotation about the drone's
         * Y-axis (pitch rate) and the sensor's ωy is rotation about
         * the drone's X-axis (roll rate).  Swap them here so the
         * gyro feed stays consistent with the swapped Euler angles —
         * otherwise the dynamics module's Lambda^-1 step would mix
         * mismatched-axis rates with re-labelled angles.  ωz is
         * unchanged because the sensor's Z-axis still aligns with
         * the drone's Z-axis after a yaw-only rotation.
         */
        sh2_Gyroscope_t *g = &sensorValue.un.gyroscope;
        s_omega_x = g->y;   /* drone roll rate  ← IMU ωy */
        s_omega_y = g->x;   /* drone pitch rate ← IMU ωx */
        s_omega_z = g->z;

        /* Freshness stamp for the staleness failsafe. */
        s_imu_last_update_ms = timer_get_ms();
        s_imu_ever_received  = true;
    }
    else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        sh2_Accelerometer_t *a = &sensorValue.un.linearAcceleration;
        /* Predict step at the BNO085 linear-accel report rate (100 Hz).    */
        kalman_z_predict(a->x, a->y, a->z);

        /* Tag "A" — body-frame linear acceleration (gravity removed).
         * Combined with the most recent R-line quaternion this becomes
         * a_z_world, which is the filter's predict-step input.             */
        // PRINTF("A: %s%.6f, %s%.6f, %s%.6f\r\n",
        //        SIGN_STR(a->x), ABS_F(a->x),
        //        SIGN_STR(a->y), ABS_F(a->y),
        //        SIGN_STR(a->z), ABS_F(a->z));
    }
}

/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitHardware();

    // debug_uart_init(DEBUG_UART_BAUDRATE);

    rc_init(RC_Callback);
    
    dshot_init(&esc, ESC_Callback);

    timer_init(&timer_0);
    timer_attach_callback(&timer_0, timer_0_callback);

    // debug_uart_print("Initialization complete. Starting scheduler...\r\n");

    /* Create Queue to hold the parsed channels data (buffer size of 5 frames) */
    rcChannelQueue = xQueueCreate(5, sizeof(fs_ia6b_channels_t));
    escChannelQueue = xQueueCreate(5, sizeof(dshotTelemetry_t));

    if (rcChannelQueue == NULL || escChannelQueue == NULL)
    {
        // debug_uart_print("Queue creation failed!\r\n");
        while (1);
    }

    if (xTaskCreate(RCParserTask, "rc_task", configMINIMAL_STACK_SIZE + 50, &esc, RC_task_PRIORITY, &rcParserTaskHandle) !=
        pdPASS)
    {
        // debug_uart_print("Task creation failed (rc)!\r\n");
        while (1)
            ;
    }

    if (xTaskCreate(ESCTelemetryTask , "esc_task", configMINIMAL_STACK_SIZE + 50, &esc, RC_task_PRIORITY, &escParserTaskHandle) !=
        pdPASS)
    {
        // debug_uart_print("Task creation failed (esc)!\r\n");
        while (1)
            ;
    }

    if (xTaskCreate(DSHOTGeneratorTask , "motor_task", configMINIMAL_STACK_SIZE + 200, &esc, MOTOR_task_PRIORITY, &dshotGeneratorTaskHandle) !=
        pdPASS)
    {
        // debug_uart_print("Task creation failed (motor)!\r\n");
        while (1)
            ;
    }

    if (xTaskCreate(ControlLoopTask, "ctrl_task", 1024, &esc, CONTROL_task_PRIORITY, &controlLoopTaskHandle) !=
        pdPASS)
    {
        // debug_uart_print("Task creation failed (control)!\r\n");
        while (1)
            ;
    }

    if (xTaskCreate(SensorTask, "Sensor_task", 1024, NULL,
                    sensor_task_PRIORITY, &sensorTaskHandle) != pdPASS)
    {
       // PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }

    bno_08x_init(&imu, IMU_Update_Callback); /* Initialize the IMU*/


    if (vlx_init(&vlx) != kStatus_Success)
    {
        // PRINTF("VLX init failed!\r\n");
    }


    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief PID speed-control loop — runs at PID_FREQUENCY_HZ (100 Hz).
 *   Inputs  : s_latest_rc  (written by RCParserTask on every valid frame)
 *             esc->motor[i] telemetry (written by ESCTelemetryTask)
 *   Outputs : s_motor_cmd[i]  (consumed by DSHOTGeneratorTask at 400 Hz)
 *
 */
static void ControlLoopTask(void *pvParameters)
{
    dshotSystem_t *esc = (dshotSystem_t *)pvParameters;
    dshotMotor_t  *motors[MAX_SUPPORTED_MOTORS] =
        { &esc->motor2, &esc->motor3, &esc->motor0, &esc->motor1 };

    pid_state_t pid[MAX_SUPPORTED_MOTORS];
   
    pid_init(&pid[0], 0.00010, 0.0080, 0.000003, PID_I_MIN, PID_I_MAX, PID_ERROR_DEADBAND_RAD_S);
    pid_init(&pid[1], 0.00012, 0.0080, 0.000003, PID_I_MIN, PID_I_MAX, PID_ERROR_DEADBAND_RAD_S);
    pid_init(&pid[2], 0.00012, 0.0080, 0.000004, PID_I_MIN, PID_I_MAX, PID_ERROR_DEADBAND_RAD_S);
    pid_init(&pid[3], 0.00010, 0.0080, 0.000004, PID_I_MIN, PID_I_MAX, PID_ERROR_DEADBAND_RAD_S);


    uint32_t last_seq[MAX_SUPPORTED_MOTORS]      = {0};
    uint8_t  stale_ticks[MAX_SUPPORTED_MOTORS]   = {0};

    const uint8_t kStaleFailsafeTicks = 5;   /* 5 * 10 ms = 50 ms */

    bool  was_disarmed_prev_tick = true;
    float z_ref_origin_m         = 0.0f;
    float yaw_ref_origin_rad     = 0.0f;

    TickType_t      xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod      = pdMS_TO_TICKS(1000U / (uint32_t)PID_FREQUENCY_HZ);

    for (;;)
    {
  
        vTaskDelayUntil(&xLastWakeTime, xPeriod);


        if (!esc->armed)
        {
            for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; ++i)
                s_motor_cmd[i] = 0U;
            continue;
        }

        bool ch7_disarmed     = (s_latest_rc.CH7.u16 < RC_CH7_ENABLE_THRESHOLD_US);
        bool tilt_failsafe    = s_tilt_failsafe_latched;
        bool imu_stale        = !s_imu_ever_received ||
                                ((timer_get_ms() - s_imu_last_update_ms) >
                                 IMU_STALE_TIMEOUT_MS);

        if (ch7_disarmed) {
            s_tilt_failsafe_latched = false;
        }

        if (ch7_disarmed || tilt_failsafe || imu_stale)
        {
            for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; ++i)
            {
                float measured_omega = dshot_motor_omega_rad_s(motors[i]);

                pid[i].integral_err  = 0.0f;
                pid[i].prev_measured = measured_omega;
                pid[i].last_output   = 0.0f;

                s_motor_cmd[i]       = 0U;
                s_motor_target_omega[i] = 0.0f;

                last_seq[i]    = motors[i]->telemetry_seq;
                stale_ticks[i] = 0;
            }
            was_disarmed_prev_tick = true;
            continue;
        }

        

        if (was_disarmed_prev_tick) {
            z_ref_origin_m     = kalman_z_get_altitude();
            yaw_ref_origin_rad = s_yaw;
            was_disarmed_prev_tick = false;
        }

        float roll_ref  = (((float)s_latest_rc.CH1.u16 - RC_PULSE_MID_US) /
                           RC_PULSE_HALF_US) * ATTITUDE_REF_MAX_RAD;
        float pitch_ref = (((float)s_latest_rc.CH3.u16 - RC_PULSE_MID_US) /
                           RC_PULSE_HALF_US) * ATTITUDE_REF_MAX_RAD;
        float z_stick   = (((float)s_latest_rc.CH2.u16 - RC_PULSE_MIN_US) /
                           RC_PULSE_RANGE_US) * Z_REF_MAX_M;

        /* Clamp stick-derived offsets so an RC glitch can't command an extreme. */
        if (roll_ref  >  ATTITUDE_REF_MAX_RAD) roll_ref  =  ATTITUDE_REF_MAX_RAD;
        if (roll_ref  < -ATTITUDE_REF_MAX_RAD) roll_ref  = -ATTITUDE_REF_MAX_RAD;
        if (pitch_ref >  ATTITUDE_REF_MAX_RAD) pitch_ref =  ATTITUDE_REF_MAX_RAD;
        if (pitch_ref < -ATTITUDE_REF_MAX_RAD) pitch_ref = -ATTITUDE_REF_MAX_RAD;
        if (z_stick   >  Z_REF_MAX_M)          z_stick   =  Z_REF_MAX_M;
        if (z_stick   <  0.0f)                 z_stick   =  0.0f;

        float z_ref   = z_ref_origin_m + z_stick;
        float yaw_ref = yaw_ref_origin_rad;

        float roll  = s_roll;
        float pitch = s_pitch;
        float yaw   = s_yaw;
        float wx = s_omega_x;
        float wy = s_omega_y;
        float wz = s_omega_z;

        float sr = sinf(roll);
        float cr = cosf(roll);
        float sp = sinf(pitch);
        float cp = cosf(pitch);
        float inv_cp = (fabsf(cp) > 1.0e-3f) ? (1.0f / cp) : 0.0f;
        float tp     = sp * inv_cp;

        float droll  = wx + tp * (sr * wy + cr * wz);
        float dpitch =      cr * wy - sr * wz;
        float dyaw   = inv_cp * (sr * wy + cr * wz);

        float states[8] = {
            kalman_z_get_altitude(),  /* states[0] = z      */
            kalman_z_get_velocity(),  /* states[1] = dz     */
            roll,                     /* states[2] = roll   */
            pitch,                    /* states[3] = pitch  */
            yaw,                      /* states[4] = yaw    */
            droll,                    /* states[5] = droll  */
            dpitch,                   /* states[6] = dpitch */
            dyaw                      /* states[7] = dyaw   */
        };

        /* Refresh the cached trig used by both SMC stages. */
        dynamics_update_trig(roll, pitch, yaw);


        float w_meas_dyn[4] = {
            dshot_motor_omega_rad_s(motors[1]),  /* CH1 motor (w0) */
            dshot_motor_omega_rad_s(motors[2]),  /* CH2 motor (w1) */
            dshot_motor_omega_rad_s(motors[0]),  /* CH3 motor (w2) */
            dshot_motor_omega_rad_s(motors[3]),  /* CH4 motor (w3) */
        };

        float U[4];
        U[0] = dynamics_compute_U0(states, z_ref);

        float eta_ref[3] = { roll_ref, pitch_ref, yaw_ref };
        float U_att[3];
        dynamics_compute_attitude_control(states, eta_ref, w_meas_dyn, U_att);
        U[1] = U_att[0];
        U[2] = U_att[1];
        U[3] = U_att[2];


        float w_target_dyn[4];
        dynamics_compute_dw(w_target_dyn, U);

        float target_omega_per_index[MAX_SUPPORTED_MOTORS];
        target_omega_per_index[1] = w_target_dyn[0];  /* CH1 motor */
        target_omega_per_index[2] = w_target_dyn[1];  /* CH2 motor */
        target_omega_per_index[0] = w_target_dyn[2];  /* CH3 motor */
        target_omega_per_index[3] = w_target_dyn[3];  /* CH4 motor */

        for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; ++i)
        {
            float target_omega = target_omega_per_index[i];

        
            s_motor_target_omega[i] = target_omega;


            float measured_omega = dshot_motor_omega_rad_s(motors[i]);


            if (target_omega < TARGET_OFF_THRESHOLD_RAD_S) {
                pid[i].integral_err  = 0.0f;
                pid[i].prev_measured = measured_omega;
                pid[i].last_output   = 0.0f;
                s_motor_cmd[i]       = 0U;       /* true motor off */
                last_seq[i]          = motors[i]->telemetry_seq; /* re-baseline */
                stale_ticks[i]       = 0;
                continue;
            }

           
            uint32_t cur_seq = motors[i]->telemetry_seq;
            if (cur_seq == last_seq[i]) {
                if (stale_ticks[i] < 0xFFU) stale_ticks[i]++;
                if (stale_ticks[i] >= kStaleFailsafeTicks) {
                    s_motor_cmd[i] = (uint16_t)DSHOT_MIN_THROTTLE;
                }
                continue;
            }

            last_seq[i]    = cur_seq;
            stale_ticks[i] = 0;

     
            float pid_out = pid_compute(&pid[i],
                                        target_omega,
                                        measured_omega,
                                        CONTROL_LOOP_PERIOD_S);


            float t_norm = pid_out;
            if (t_norm < 0.0f) t_norm = 0.0f;
            uint16_t cmd  = (uint16_t)((float)DSHOT_MIN_THROTTLE +
                                        t_norm * (float)DSHOT_RANGE);

            s_motor_cmd[i] = cmd;
        }
    }
}


static void DSHOTGeneratorTask(void *pvParameters)
{
    dshotSystem_t *esc = (dshotSystem_t *)pvParameters;
    dshotMotor_t *motors[MAX_SUPPORTED_MOTORS] =
        { &esc->motor2, &esc->motor3, &esc->motor0, &esc->motor1 };
    uint8_t next_telemetry_motor = 0;

    __disable_irq();
    dshot_startup_sequence(esc);
    __enable_irq();

    timer_start(&timer_0);

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == pdTRUE) {
            /* Always send a DSHOT update; only request telemetry if the
               previous response has fully resolved. */
            uint8_t tm_motor = 0xFF;
            if (!s_response_in_flight) {
                tm_motor = next_telemetry_motor;
                next_telemetry_motor = (next_telemetry_motor + 1) % MAX_SUPPORTED_MOTORS;

                if (escParserTaskHandle != NULL) {
                    (void)xTaskNotifyStateClear(escParserTaskHandle);
                }


                esc_uart_flush();
                esc_start_dma_rx(g_esc_rxBuffer, DSHOT_TELEMETRY_FRAME_SIZE);

                s_pending_motor_id   = tm_motor;
                s_response_in_flight = true;
            }
            __disable_irq();
            for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; ++i) {
                motors[i]->dshot_control.requestTelemetry_b = (i == tm_motor);
                dshot_send_frame(motors[i]);
                delay_blocking_us(20); // Avoid back-to-back DMA transcation.
                motors[i]->dshot_control.requestTelemetry_b = false;
            }
            __enable_irq();

            if (esc->armed) {
                for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; ++i) {
                    motors[i]->dshot_control.throttle_u16 = s_motor_cmd[i];
                }
            } else {
                for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; ++i) {
                    motors[i]->dshot_control.throttle_u16 = 0;
                }
            }

        }
    }
}


/*!
 * @brief Task responsible for parsing the received RC frame.
 */
static void RCParserTask(void *pvParameters)
{
    fs_ia6b_frame_t current_rc_frame;
    //dshotSystem_t *esc = (dshotSystem_t *)pvParameters;
    TickType_t timeout_ticks = pdMS_TO_TICKS(RC_TIMEOUT_MS);

    rc_start_dma_rx(g_rc_rxBuffer, FS_IA6B_FRAME_SIZE);

    for (;;)
    {

        /* Wait to be notified by the EDMA RC ISR */
        if (ulTaskNotifyTake(pdTRUE, timeout_ticks) == pdTRUE)
        {   
            /* Check if the frame passes CRC and is successfully parsed */
            if (rc_parse_frame(g_rc_rxBuffer, &current_rc_frame) == kRC_StatusSucces)
            {

                s_latest_rc = current_rc_frame.channels;

            }
            else 
            {
                
                if (rc_sync(RC_TIMEOUT_MS) == kRC_StatusFail) {
                    // TODO: Trigger drone failsafe (e.g., drop throttle, level out)
                }
            }
            /* Clean up and restart DMA reception for the next frame */
            rc_start_dma_rx(g_rc_rxBuffer, FS_IA6B_FRAME_SIZE);

        } else {
            //Handle remote control connection timeout (trigger failsafe)
        }
    }
}


/*!
 * @brief Task responsible for parsing the received ESC telemetry frame.
 */
static void ESCTelemetryTask(void *pvParameters)
{
    dshotSystem_t *esc = (dshotSystem_t *)pvParameters;
    dshotMotor_t *motors[MAX_SUPPORTED_MOTORS] =

        { &esc->motor2, &esc->motor3, &esc->motor0, &esc->motor1 };
    const TickType_t kFrameTimeout = pdMS_TO_TICKS(2);


    uint8_t payload[15];

    for (;;) {
        /* Sleep until the generator says "I just armed RX for motor X". */
        while (!s_response_in_flight) {
            vTaskDelay(1);
        }

        uint8_t mid = s_pending_motor_id;
        if (ulTaskNotifyTake(pdTRUE, kFrameTimeout) == pdTRUE) {
            if (mid < MAX_SUPPORTED_MOTORS &&
                dshot_process_telemetry(motors[mid], g_esc_rxBuffer) == kStatus_Success) {
                if (dshot_telemetry_plausible(&motors[mid]->dshot_telemtry)) {
                    /* Layer-3: passed CRC AND plausibility -> accept. */
                    motors[mid]->stats.tm_ok++;
                    motors[mid]->stats.tm_consecutive_fail = 0;

                    /* Publish freshness: ControlLoopTask uses this counter */
                    motors[mid]->telemetry_seq++;

                    /* Arm-on-telemetry: as soon as every motor has been
                     * seen at least once, flip the global armed flag.
                     * The DSHOT generator is the consumer of armed and
                     * keeps throttle = 0 until then.                      */
                    if (!esc->armed) {
                        esc->seen_mask |= (uint8_t)(1U << mid);
                        if (esc->seen_mask ==
                            (uint8_t)((1U << MAX_SUPPORTED_MOTORS) - 1U)) {
                            esc->armed = true;
                            // debug_uart_print("ARMED: all motors confirmed.\r\n");
                        }
                    }

                    {
                        
                        uint32_t ts_ms  = timer_get_ms();
                        float    omega  = dshot_motor_omega_rad_s(motors[mid]);
                        float    target = s_motor_target_omega[mid];
                        uint16_t thr    = motors[mid]->dshot_control.throttle_u16;

                        memcpy(&payload[0],  &ts_ms,  sizeof(ts_ms));   /* 4 */
                        payload[4] = mid;                               /* 1 */
                        memcpy(&payload[5],  &omega,  sizeof(omega));   /* 4 */
                        memcpy(&payload[9],  &target, sizeof(target));  /* 4 */
                        memcpy(&payload[13], &thr,    sizeof(thr));     /* 2 */

                        // (void)debug_uart_send_frame(
                        //     DEBUG_UART_FRAME_ID_TELEMETRY,
                        //     payload, sizeof(payload));
                    }

                } else {
                    /* Frame parsed but values are out of range: rare CRC
                     * collision, or the ESC is misbehaving. Reject. */
                    motors[mid]->stats.tm_implausible++;
                    motors[mid]->stats.tm_consecutive_fail++;
                    motors[mid]->dshot_telemtry.valid_b = false;
                }
            } else if (mid < MAX_SUPPORTED_MOTORS) {
                /* CRC failed or buffer was misaligned. */
                motors[mid]->stats.tm_crc_fail++;
                motors[mid]->stats.tm_consecutive_fail++;
                motors[mid]->dshot_telemtry.valid_b = false;
            }
        } else {
            /* Layer-2 timeout recovery: cancel the in-flight EDMA RX
             * and flush the LPUART RX FIFO so any late-arriving bytes
             * cannot leak into the NEXT motor's response. */
            esc_uart_abort_rx();
            esc_uart_flush();
            if (mid < MAX_SUPPORTED_MOTORS) {
                motors[mid]->stats.tm_timeouts++;
                motors[mid]->stats.tm_consecutive_fail++;
                motors[mid]->dshot_telemtry.valid_b = false;
            }
        }

        /* Mark this round resolved -- generator can issue the next
         * request on the next timer tick. */
        s_response_in_flight = false;
    }
}

/*!
 * @brief Task responsible for processing the IMU reports.  Wakes on the
 *        IMU HINT falling edge, drains all pending packets from the BNO085,
 *        and lets sh2_sensor_callback() print the decoded rotation vector
 *        and gyroscope samples.
 */
static void SensorTask(void *pvParameters)
{
    (void)pvParameters;

    static bool sensors_configured = false;
    uint32_t    notif_value        = 0;
    status_t    result;


    vTaskDelay(pdMS_TO_TICKS(100));


    kalman_z_init(KZ_Q, KZ_R_Z, KZ_DT);
    g_kalman_ready = true;


    result = bno_08x_start(sh2_sensor_callback);
    if (result != kStatus_Success) {
        //PRINTF("BNO085 SH2 session failed to open\r\n");
        vTaskSuspend(NULL);
    }


    for (;;)
    {
        notif_value = 0;
        xTaskNotifyWait(/* ulBitsToClearOnEntry */ 0,
                        /* ulBitsToClearOnExit  */ IMU_NOTIFY_BIT,
                        &notif_value,
                        pdMS_TO_TICKS(100));

        if (notif_value & IMU_NOTIFY_BIT)
        {
            /* Drain all packets the BNO has queued.  Bound the loop so a
             * stuck-LOW HINT can't hang SensorTask.                       */
            int drain_budget = 16;
            do {
                sh2_service();
            } while (gpio_read_input(&imu.gpio_event) == 0 && --drain_budget > 0);
        }


        status_t vlx_status = vlx_read(&vlx);

        /* First IMU reset is the trigger to enable the sensor reports. */
        if (!sensors_configured && bno_08x_reset_occurred()) {
            bno_08x_configure_sensors();
            vTaskDelay(pdMS_TO_TICKS(200));   /* let reports start flowing */
            sensors_configured = true;
        }


        if (vlx_status == kStatus_Success &&
            vlx.data.status == VLX_STATUS_OK)
        {
            const float32_t z_raw_m = (float32_t)vlx.data.distance_mm * 0.001f;
            kalman_z_update(z_raw_m);
        }

    }
}



