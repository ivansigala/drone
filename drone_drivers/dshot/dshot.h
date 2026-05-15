/*
 * dshot.h
 *
 * Created on: Enero, 2026
 * Author: diego
 */

#pragma once

#ifndef DSHOT_H_
#define DSHOT_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef MCXN947
#include "fsl_common.h"
#include "uart_driver_mcxn947.h"
#include "dma_driver_mcxn947.h"
#include "pwm_driver_mcxn947.h"
#include "app.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Dshot defines */
#define MAX_SUPPORTED_MOTORS 4

#define DSHOT_RING_BUFFER_SIZE 32

#define DSHOT_TELEMETRY_FRAME_SIZE 10

/* DShot frames consist of 16 bits */
#define DSHOT_FRAME_SIZE       16
/* Buffer size includes the 16 bits plus a reset end-of-frame period */
#define DSHOT_DMA_BUFFER_SIZE  18

#define DSHOT_150_FREQ 150000U
#define DSHOT_300_FREQ 300000U
#define DSHOT_600_FREQ 600000U
#define DSHOT_1200_FREQ 1200000U

/* * DShot Timing Definitions (Example for DShot600)
 * These values depend on your PWM source clock frequency.
 * For a 100MHz clock, a 1.67us period is ~167 ticks.
 * Bit 0: ~37% duty, Bit 1: ~75% duty.
 */
#define DSHOT_600_BIT_0        94
#define DSHOT_600_BIT_1        188

/* Startup & inter-frame timing */
#define DSHOT_STARTUP_TIME_MS           300
/* Time for a 10-byte telemetry packet at 115200 baud 8N1: ~868 µs, rounded to 1 ms */
#define DSHOT_INTER_FRAME_DELAY_MS       5 

#define DSHOT_MIN_THROTTLE              (48)
#define DSHOT_MAX_THROTTLE              (2047)
#define DSHOT_3D_FORWARD_MIN_THROTTLE   (1048)
#define DSHOT_RANGE                     (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

#define DSHOT_TELEMETRY_NOEDGE          (0xfffe)
#define DSHOT_TELEMETRY_INVALID         (0xffff)

#define MIN_GCR_EDGES                   (7)
#define MAX_GCR_EDGES                   (22)

/*
 * eRPM -> mechanical rad/s conversion
 * -----------------------------------
 *
 * On the wire dshotTelemetry_t::erpm_u16 holds  electrical_rpm / 100.
 * Mechanical RPM = electrical_rpm / pole_pairs, and rad/s = RPM*2*pi/60.
 * Combined:
 *
 *      omega [rad/s] = erpm_u16 * 100 / pole_pairs * (2*pi / 60)
 *                    = erpm_u16 * (10*pi / 3) / pole_pairs
 *
 * Override DSHOT_MOTOR_POLE_PAIRS for your motor (most drone outrunners
 * in the 2204..2812 class are 7 pole-pair / 14-pole). The conversion
 * factor is pre-folded into a single float so each call site is one
 * float multiply.
 */
#ifndef DSHOT_MOTOR_POLE_PAIRS
#define DSHOT_MOTOR_POLE_PAIRS  7
#endif

#define DSHOT_ERPM_TO_RAD_S \
    ((100.0f * 2.0f * 3.14159265358979323846f) / \
     (60.0f * (float)DSHOT_MOTOR_POLE_PAIRS))

/*!
 * @brief Convert a raw dshotTelemetry_t::erpm_u16 reading to the
 *        rotor's mechanical angular velocity in rad/s.
 *
 *  Spin direction is NOT encoded in the telemetry frame -- the value
 *  returned here is always >= 0. If your control law needs signed
 *  angular velocity (e.g. 3D mode reversing motors), pair this with
 *  the commanded direction kept by the application.
 */
static inline float dshot_erpm_to_rad_s(uint16_t erpm_u16)
{
    return (float)erpm_u16 * DSHOT_ERPM_TO_RAD_S;
}

/* Forward declaration for the convenience helper below; the full
 * definition needs the dshotMotor_t struct, which is defined further
 * down in this header. */
struct dshotMotor_s;

/*!
 * @brief Convenience: read the motor's last validated mechanical
 *        angular velocity, in rad/s. Returns 0 if the most recent
 *        frame failed validation (i.e. valid_b is false).
 */
float dshot_motor_omega_rad_s(const struct dshotMotor_s *motor);

/*******************************************************************************
 * Telemetry
 ******************************************************************************/

/*
 * Per-motor telemetry health counters. Bumped by the telemetry task on
 * every request-response cycle so the application can decide when a
 * motor's feedback should no longer be trusted by the control loop.
 *
 *   tm_ok                  : frames that passed CRC + plausibility
 *   tm_crc_fail            : frames that arrived but failed CRC
 *   tm_timeouts            : the ESC never responded within the
 *                            configured deadline
 *   tm_implausible         : frames that passed CRC but failed
 *                            plausibility (out-of-range voltage / temp)
 *   tm_consecutive_fail    : consecutive failures of any kind; resets
 *                            on the next ok. Use this to drive a
 *                            failsafe gate in the control loop.
 */
typedef struct dshotTelemetryStats_s {
    uint32_t tm_ok;
    uint32_t tm_crc_fail;
    uint32_t tm_timeouts;
    uint32_t tm_implausible;
    uint32_t tm_consecutive_fail;   /* resets on any ok */
} dshotTelemetryStats_t;

typedef struct dshotTelemetry_s {
	int8_t temperature_u8;      // Temperature in °C
	uint16_t voltage_cv_u16;     // Voltage in centivolts (10.00V = 1000)
	uint16_t current_ca_u16;     // Current in centiamps (10.00A = 1000)
	uint16_t consumption_mah_u16;// Consumption in mAh
	uint16_t erpm_u16;           // Electrical RPM / 100 (100 = 10000 eRPM)
	bool valid_b;              // True if the last packet was valid
} dshotTelemetry_t;


typedef struct dshotControl_s {
    uint16_t throttle_u16;
    bool requestTelemetry_b;
} dshotControl_t;


/*******************************************************************************
 * Motor
 ******************************************************************************/
typedef struct dshotMotor_s {

	dshotTelemetry_t      dshot_telemtry;
	dshotControl_t        dshot_control;
	dshotTelemetryStats_t stats;
	dma_ctrl_t            dma;
	pwm_ctrl_t            pwm;
	uint8_t               dma_id;
	uint8_t               motor_id;

	/*
	 * Monotonic counter, bumped by the ESCTelemetryTask whenever a
	 * frame for THIS motor passes CRC + plausibility. The control
	 * loop snapshots its last-seen value to detect "no fresh data
	 * since the previous tick" and avoid re-running the PID against
	 * a stale measurement (which would otherwise drive the throttle
	 * upward on every dropped frame -- the source of the cyan-trace
	 * spikes you saw at constant target).
	 *
	 * Volatile because it crosses the task boundary (writer:
	 * ESCTelemetryTask, reader: ControlLoopTask). 32-bit aligned ⇒
	 * atomic on Cortex-M33; no further sync needed.
	 */
	volatile uint32_t     telemetry_seq;

} dshotMotor_t;

/*******************************************************************************
 * Wraper
 ******************************************************************************/
/**
 * @brief Estructura global del sistema DShot.
 * Contiene la memoria real para todos los motores y la UART.
 */
typedef struct dshotSystem_s {
    uart_ctrl_t      telemetry_uart;
    dshotMotor_t     motor0;
    dshotMotor_t     motor1;
    dshotMotor_t     motor2;
    dshotMotor_t     motor3;

    /*
     * Telemetry-confirmed arming state.
     *
     *   seen_mask : bit i is set the first time motor i returns a
     *               valid (CRC-passing) telemetry frame.
     *   armed     : becomes true the first cycle in which seen_mask
     *               covers every motor (== (1<<MAX_SUPPORTED_MOTORS)-1).
     *
     * The application task that drives DSHOT MUST hold all motor
     * throttles at 0 while !armed, regardless of what the control
     * loop wants. This guarantees the ESCs received their full
     * arming window AND that we have a working telemetry path on
     * every motor before the controller is allowed to spin them up.
     *
     * Volatile because they're written by the telemetry task and
     * read by the DSHOT generator task.
     */
    volatile bool    armed;
    volatile uint8_t seen_mask;

} dshotSystem_t;

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Initializes all needed for dshot
 * @param dshot system Pointer.
 * @param telemetry_callback_ptr Callback for the telemetry UART DMA.
 * @return kStatus_Success if initialization is successful, kStatus_Fail otherwise.
 */
status_t dshot_init(dshotSystem_t *sys, void* telemetry_callback_ptr);

/*!
 * @brief Initializes both the PWM and DMA peripherals for a specific DShot motor.
 * @param motor Pointer to the motor control structure.
 * @return kStatus_Success if initialization is successful, kStatus_Fail otherwise.
 */
status_t dshot_motor_init(dshotMotor_t *motor);

/*!
 * @brief Initializes the UART peripheral used for DShot telemetry.
 * @param uart_config_ptr Pointer to the uart configuration.
 */
void dshot_telemetry_init(uart_ctrl_t* uart_config_ptr);

/*!
 * @brief Parses the UART ring buffer and assigns telemetry to the current motor.
 * @param current_motor Pointer to the motor that is currently expected to send telemetry.
 * @param buffer Pointer to the UART ring buffer.
 */
status_t dshot_process_telemetry(dshotMotor_t *current_motor, uint8_t *buffer);

/*
 * @brief Starts the DMA receive operation for DShot telemetry.
 * @param buffer Pointer to the buffer where received data will be stored.
 * @param length The number of bytes to receive.
 * @return kStatus_Success if the operation is successful, kStatus_Fail otherwise.
 */
status_t esc_start_dma_rx(uint8_t *buffer, uint32_t length);

/*!
 * @brief Initializes the eDMA engine for DShot operations.
 * @param dma The DMA peripheral struct attributes.
 */
void dshot_dma_init(dma_ctrl_t *dma);

/*!
 * @brief Generates the 16-bit DShot packet with CRC.
 * @param throttle 11-bit throttle value (48-2047 valid, 0-47 reserved).
 * @param request_telemetry Telemetry bit flag.
 * @return 16-bit formatted DShot packet.
 */
uint16_t dshot_prepare_packet(uint16_t throttle, bool request_telemetry);

/*!
 * @brief Encodes a throttle value into a DMA buffer and triggers the transfer.
 * @param motor Pointer to the motor control structure for the specific motor.
 */
void dshot_send_frame(dshotMotor_t *motor);

/*!
 * @brief Sends DShot frames to all motors.
 * @param sys Pointer to the DShot system structure.
 */
void dshot_send_frame_all(dshotSystem_t *sys);

/*!
 * @brief Updates the CRC8 value with a new byte.
 * @param crc The current CRC value.
 */
uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);

/*!
 * @brief Calculates the CRC8 for a given buffer.
 * @param Buf Pointer to the data buffer.
 * @param BufLen Length of the data buffer.
 * @return The calculated CRC8 value.
 */
uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);

/*!
 * @brief Performs the startup sequence for DShot motors.
 * @param esc Pointer to the DShot system structure.
 */
void dshot_startup_sequence(dshotSystem_t *esc);

/*!
 * @brief Drain any bytes sitting in the ESC telemetry UART RX FIFO and
 *        clear all error/idle status flags.
 *
 *  Called BEFORE arming a fresh EDMA RX so a stray byte left over from
 *  a previous late response does not get latched as the first byte of
 *  the new frame.
 */
void esc_uart_flush(void);

/*!
 * @brief Cancel an in-flight EDMA RX on the ESC telemetry UART.
 *
 *  Used by the timeout-recovery path. After calling this the caller
 *  should esc_uart_flush() and only re-arm with esc_start_dma_rx() on
 *  the next request cycle.
 */
status_t esc_uart_abort_rx(void);

/*!
 * @brief Cheap plausibility check on a parsed telemetry record.
 *
 *  Catches obviously-corrupted frames that happened to pass CRC (rare
 *  but possible). Bounds are intentionally loose -- tighten them per
 *  your battery / motor in the application if you want.
 *
 *  @return true if the record looks plausible.
 */
bool dshot_telemetry_plausible(const dshotTelemetry_t *t);

#endif /* DSHOT_H_ */
