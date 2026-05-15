/*
 * rc_fsia6B.h
 * Author: Diego
 * Created on: 6, April 2026
 */

#ifndef RC_FSIA6B_H
#define RC_FSIA6B_H

#include "FreeRTOS.h"
#include "task.h"

/* Especific defines for MCXN947 */
#ifdef MCXN947
#include "uart_driver_mcxn947.h"
#endif

#define RC_RING_BUFFER_SIZE 1024
#define FS_IA6B_FRAME_SIZE 32
#define FS_IA6B_START_BYTE 0x20
#define RC_CHANNEL_MIN 800    /* Minimum PWM value (microseconds) */
#define RC_CHANNEL_MAX 2200   /* Maximum PWM value (microseconds) */
#define RC_CHANNEL_CENTER 1500 /* Center/neutral PWM value */
#define RC_TIMEOUT_MS 50    /* Timeout for waiting for a valid frame */

typedef union
{
	struct u8_
	{
		uint8_t low;
		uint8_t high;
	}u8;

	uint16_t u16;

}RC_Channel_t;

/* FS-iA6B Channels (14 channels total) */
typedef struct
{
	RC_Channel_t CH1;
	RC_Channel_t CH2;
	RC_Channel_t CH3;
	RC_Channel_t CH4;
	RC_Channel_t CH5;
	RC_Channel_t CH6;
	RC_Channel_t CH7;
	RC_Channel_t CH8;
	RC_Channel_t CH9;
	RC_Channel_t CH10;
	RC_Channel_t CH11;
	RC_Channel_t CH12;
	RC_Channel_t CH13;
	RC_Channel_t CH14;
} fs_ia6b_channels_t;

typedef struct __attribute__((packed))
{
    uint8_t length;                 /* Byte 0 - Length (always 0x20) */
    uint8_t command;                /* Byte 1 - Command (0x40 for channels) */
    fs_ia6b_channels_t channels;    /* Bytes 2-29 - 14 channels × 2 bytes each */
    uint16_t checksum;              /* Bytes 30-31 - 16-bit Checksum */
} fs_ia6b_frame_t;


typedef enum
{
	kRC_StatusSucces = 0,
	kRC_StatusFail,
}rc_status;

typedef enum
{
	kRC_EventNewData,
	kRC_EventTimeout
}rc_event_t;

/* Function Prototypes */

rc_status rc_init(void* func_ptr);
rc_status uart_sync_rx(uart_ctrl_t *ctrl, uint32_t timeout_ms);
rc_status rc_sync(uint32_t timeout_ms);
rc_status rc_start_dma_rx(uint8_t *buffer, uint32_t length);
rc_status rc_parse_frame(uint8_t *buffer, fs_ia6b_frame_t *parsed_frame);

#endif /* RC_FSIA6B_H */