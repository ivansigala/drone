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
#include "fsl_common.h"

#ifdef MCXN947
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

/*******************************************************************************
 * Telemetry
 ******************************************************************************/

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

	dshotTelemetry_t dshot_telemtry;
	dshotControl_t   dshot_control;
	dma_ctrl_t       dma;
	pwm_ctrl_t       pwm;
	uint8_t		     dma_id;
	uint8_t          motor_id;

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

#endif /* DSHOT_H_ */
