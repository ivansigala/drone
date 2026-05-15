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
#include "rc_fsia6B.h"
#include "dshot.h"
#include "pid.h"
#include "timer_driver_mcxn947.h"
#include "debug_uart_mcxn947.h"
#include "gpio_driver_mcxn947.h"

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
#define PID_I_MIN      -70.0f
#define PID_I_MAX       70.0f

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

/*
 * Below this target speed, the control loop short-circuits to "motor
 * off" -- it sends DSHOT throttle = 0 (true off, motor freewheels) and
 * resets the PID state to keep the integrator from winding up. The
 * threshold is set above PID_ERROR_DEADBAND_RAD_S so that the deadband
 * around a non-zero setpoint never accidentally trips the off-case.
 */
#define TARGET_OFF_THRESHOLD_RAD_S  20.0f

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Task priorities. */
#define RC_task_PRIORITY 4
#define ESC_task_PRIORITY 4
#define MOTOR_task_PRIORITY 4
#define CONTROL_task_PRIORITY 4
#define SENSOR_task_PRIORITY 4

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void RCParserTask(void *pvParameters);
static void ESCTelemetryTask(void *pvParameters);
static void DSHOTGeneratorTask(void *pvParameters);
static void ControlLoopTask(void *pvParameters);

void RC_Callback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData);
void ESC_Callback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData);


/*******************************************************************************
 * Variables
 ******************************************************************************/
AT_NONCACHEABLE_SECTION_INIT(uint8_t g_rc_rxBuffer[FS_IA6B_FRAME_SIZE]) = {0};
AT_NONCACHEABLE_SECTION_INIT(uint8_t g_esc_rxBuffer[DSHOT_TELEMETRY_FRAME_SIZE]) = {0};

/* Task handle for notifications */
TaskHandle_t rcParserTaskHandle      = NULL;
TaskHandle_t escParserTaskHandle     = NULL;
TaskHandle_t dshotGeneratorTaskHandle = NULL;
TaskHandle_t controlLoopTaskHandle   = NULL;

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

/* ESC Handle */
dshotSystem_t esc;

/* Global variables for telemetry data */
volatile uint8_t g_motor_id = 0;
volatile uint8_t g_erpm_low = 0;
volatile uint8_t g_erpm_high = 0;

static volatile uint8_t  s_pending_motor_id   = 0xFF;   /* 0xFF = idle */
static volatile bool     s_response_in_flight = false;

timer_ctrl_t timer_0 ={
    .timer_id = 0,
    .frequency = 400
};

timer_ctrl_t timer_1 ={
    .timer_id = 1,
    .frequency = 100
};


static gpio_ctrl_t gpio_timer_tracker = {
    .gpio_base = GPIO0,
    .port_base = PORT0,
    .pin = 23,
    .dir = gpio_output
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

/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitHardware();
    
    gpio_init(&gpio_timer_tracker);

    // GETCHAR();

    /* Take over LPUART4 for non-blocking telemetry. After this call,
     * PRINTF must NOT be used: the debug-console handle is no longer
     * consistent with the hardware state.                              */
    debug_uart_init(DEBUG_UART_BAUDRATE);

    rc_init(RC_Callback);
    
    dshot_init(&esc, ESC_Callback);

    timer_init(&timer_0);
    timer_attach_callback(&timer_0, timer_0_callback);

    debug_uart_print("Initialization complete. Starting scheduler...\r\n");

    /* Create Queue to hold the parsed channels data (buffer size of 5 frames) */
    rcChannelQueue = xQueueCreate(5, sizeof(fs_ia6b_channels_t));
    escChannelQueue = xQueueCreate(5, sizeof(dshotTelemetry_t));

    if (rcChannelQueue == NULL || escChannelQueue == NULL)
    {
        debug_uart_print("Queue creation failed!\r\n");
        while (1);
    }

    if (xTaskCreate(RCParserTask, "rc_task", configMINIMAL_STACK_SIZE + 50, &esc, RC_task_PRIORITY, &rcParserTaskHandle) !=
        pdPASS)
    {
        debug_uart_print("Task creation failed (rc)!\r\n");
        while (1)
            ;
    }

    if (xTaskCreate(ESCTelemetryTask , "esc_task", configMINIMAL_STACK_SIZE + 50, &esc, RC_task_PRIORITY, &escParserTaskHandle) !=
        pdPASS)
    {
        debug_uart_print("Task creation failed (esc)!\r\n");
        while (1)
            ;
    }

    if (xTaskCreate(DSHOTGeneratorTask , "motor_task", configMINIMAL_STACK_SIZE + 200, &esc, MOTOR_task_PRIORITY, &dshotGeneratorTaskHandle) !=
        pdPASS)
    {
        debug_uart_print("Task creation failed (motor)!\r\n");
        while (1)
            ;
    }

    /* Extra stack covers 4× pid_state_t (~32 B each) plus local floats. */
    if (xTaskCreate(ControlLoopTask, "ctrl_task", configMINIMAL_STACK_SIZE + 150, &esc, CONTROL_task_PRIORITY, &controlLoopTaskHandle) !=
        pdPASS)
    {
        debug_uart_print("Task creation failed (control)!\r\n");
        while (1)
            ;
    }
    

    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief PID speed-control loop — runs at PID_FREQUENCY_HZ (100 Hz).
 *
 * Architecture
 * ────────────
 *   Inputs  : s_latest_rc  (written by RCParserTask on every valid frame)
 *             esc->motor[i] telemetry (written by ESCTelemetryTask)
 *   Outputs : s_motor_cmd[i]  (consumed by DSHOTGeneratorTask at 400 Hz)
 *
 * Control law (per motor)
 * ───────────────────────
 *   1. Map RC CH3 (throttle stick, 800–2200 µs) to a target angular
 *      velocity in rad/s, linearly scaled to [0, MAX_MOTOR_OMEGA_RAD_S].
 *   2. Read the motor's actual omega from the last validated telemetry
 *      frame via dshot_motor_omega_rad_s().
 *   3. Run pid_compute() → normalised output in [-1, 1].
 *   4. Map output to DSHOT throttle [DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE].
 *   5. Apply a throttle deadband: if the stick is below THROTTLE_DEADBAND_NORM
 *      the motors are commanded off (throttle = 0) and the integrators are
 *      reset so there is no windup across an idle period.
 *
 * Gains (PID_KP / PID_KI / PID_KD) are compile-time constants at the top
 * of this file.  Tune them on the bench before flight.
 */
static void ControlLoopTask(void *pvParameters)
{
    dshotSystem_t *esc = (dshotSystem_t *)pvParameters;
    dshotMotor_t  *motors[MAX_SUPPORTED_MOTORS] =
        { &esc->motor0, &esc->motor1, &esc->motor2, &esc->motor3 };

    static uint32_t samples = 0;

    pid_state_t pid[MAX_SUPPORTED_MOTORS];
   
    pid_init(&pid[0], 0.00010, 0.0080, 0.000003, PID_I_MIN, PID_I_MAX, PID_ERROR_DEADBAND_RAD_S);
    pid_init(&pid[1], 0.00012, 0.0080, 0.000003, PID_I_MIN, PID_I_MAX, PID_ERROR_DEADBAND_RAD_S);
    pid_init(&pid[2], 0.00012, 0.0080, 0.000004, PID_I_MIN, PID_I_MAX, PID_ERROR_DEADBAND_RAD_S);
    pid_init(&pid[3], 0.00010, 0.0080, 0.000004, PID_I_MIN, PID_I_MAX, PID_ERROR_DEADBAND_RAD_S);
    /*
     * Per-motor "last telemetry sequence number we acted on". When the
     * current motors[i]->telemetry_seq is unchanged from the value we
     * cached here, no new measurement has arrived since the previous
     * tick -- we skip the PID for that motor and hold the previous
     * command. Avoids the throttle spike caused by stale (or
     * dshot_motor_omega_rad_s()-returned-0) measurements.
     */
    uint32_t last_seq[MAX_SUPPORTED_MOTORS]      = {0};
    uint8_t  stale_ticks[MAX_SUPPORTED_MOTORS]   = {0};

    /*
     * After this many consecutive control ticks with no fresh
     * telemetry, fall back to a safe minimum throttle. Prevents a
     * permanently dead ESC from being held at whatever the last
     * pre-fault command was.
     */
    const uint8_t kStaleFailsafeTicks = 5;   /* 5 * 10 ms = 50 ms */

    TickType_t      xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod      = pdMS_TO_TICKS(1000U / (uint32_t)PID_FREQUENCY_HZ);

    for (;;)
    {
        /* Pace the loop precisely at PID_FREQUENCY_HZ regardless of how
         * long the computation takes (as long as it stays within one period). */
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        gpio_toggle_output(&gpio_timer_tracker);

        /* Hold commands at zero until every motor has confirmed telemetry. */
        if (!esc->armed)
        {
            for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; ++i)
                s_motor_cmd[i] = 0U;
            continue;
        }

        // float ch3_us      = (float)s_latest_rc.CH3.u16;
        // float throttle_norm = (ch3_us - (float)RC_CHANNEL_MIN) /
        //                       (float)(RC_CHANNEL_MAX - RC_CHANNEL_MIN);

        samples++;

        /* Clamp to [0, 1] in case of a slightly out-of-range transmission. */
        // if (throttle_norm < 0.0f) throttle_norm = 0.0f;
        // if (throttle_norm > 1.0f) throttle_norm = 1.0f;

        
        // if (throttle_norm < THROTTLE_DEADBAND_NORM)
        // {
        //     for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; ++i)
        //     {
        //         pid[i].integral_err = 0.0f;  /* reset integral to prevent windup during idle */
        //         pid[i].prev_measured = 0.0f; /* reset derivative term to prevent spikes on re-arming */
        //         pid[i].last_output = 0.0f;   /* for telemetry, not used in control */
        //         s_motor_cmd[i] = 0U;
        //     }
        //     continue;
        // }

        float time_sec = (float)samples * CONTROL_LOOP_PERIOD_S;
        float target_omega = 600.0f + 200.0f * sinf(2.0f * 3.1415926f * 1.0f * time_sec);

        for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; ++i)
        {
            /* Publish the setpoint for this motor so the telemetry task
             * can include it in the debug-UART frame.                   */
            s_motor_target_omega[i] = target_omega;

            float measured_omega = dshot_motor_omega_rad_s(motors[i]);

            /*
             * Off-case short circuit. The unidirectional ESC's smallest
             * non-zero command (DSHOT_MIN_THROTTLE) keeps the motor
             * spinning at idle (~135 rad/s on this bench), so we cannot
             * "command zero" via PID output -- we have to bypass the
             * mapping entirely. Resetting the integrator + prev_measured
             * also prevents windup while the motor is intentionally off
             * and avoids a derivative kick on the next non-zero target.
             */
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

            /* pid_compute returns a normalised correction in [-1, 1].
             * The library uses derivative-on-measurement so a step change
             */
            float pid_out = pid_compute(&pid[i],
                                        target_omega,
                                        measured_omega,
                                        CONTROL_LOOP_PERIOD_S);

            /*
             * Unidirectional throttle mapping for ESCs that only spin one
             * way:
             *
             *   pid_out <= 0   →   t_norm = 0   →   cmd = DSHOT_MIN_THROTTLE
             *   pid_out  = +1  →   t_norm = 1   →   cmd = DSHOT_MAX_THROTTLE
             */
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
        { &esc->motor0, &esc->motor1, &esc->motor2, &esc->motor3 };
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

                /* Drain any stale notification from a callback that fired
                 * AFTER the previous round timed out. Without this, the
                 * ESC task would treat the carry-over notification as the
                 * arrival of THIS new request's response. */
                if (escParserTaskHandle != NULL) {
                    (void)xTaskNotifyStateClear(escParserTaskHandle);
                }

                /* Layer-2 hygiene: drain the LPUART RX FIFO so that any
                 * late-arriving byte from a previous response cannot get
                 * latched as the first byte of this transfer. Then arm
                 * the EDMA fresh. */
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

            /*
             * Throttle gate: ESCTelemetryTask flips esc->armed only after
             * every motor has returned at least one valid telemetry frame.
             * Until then we hold all throttles at zero so the control loop
             * cannot spin a motor that we don't yet know is present and
             * talking back.
             */
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
                /* Publish the validated channel set so ControlLoopTask can
                 * read the latest stick positions without a queue copy.
                 * Each 16-bit channel field is written atomically so no mutex is needed here.
                 */
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
        { &esc->motor0, &esc->motor1, &esc->motor2, &esc->motor3 };
    const TickType_t kFrameTimeout = pdMS_TO_TICKS(2);

    /*
     * Debug-UART telemetry payload, little-endian on the wire (15 bytes):
     *
     *   bytes 0..3    uint32  ts_ms          LPTMR0-derived timestamp [ms]
     *   byte  4       uint8   motor_id       0..MAX_SUPPORTED_MOTORS-1
     *   bytes 5..8    float32 omega_rad_s    measured angular velocity
     *   bytes 9..12   float32 target_rad_s   setpoint from ControlLoopTask
     *   bytes 13..14  uint16  throttle       last commanded DSHOT (0..2047)
     */
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
                            debug_uart_print("ARMED: all motors confirmed.\r\n");
                        }
                    }

                    /* Stream the freshly-validated sample out the framed
                     * debug UART. See `payload` declaration above for
                     * the byte-by-byte layout. Non-blocking: a full
                     * ring buffer drops the frame instead of stalling
                     * this task.                                          */
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

                        (void)debug_uart_send_frame(
                            DEBUG_UART_FRAME_ID_TELEMETRY,
                            payload, sizeof(payload));
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



