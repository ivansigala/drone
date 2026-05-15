/*
 * debug_uart_mcxn947.h
 *
 * Non-blocking, interrupt-driven debug UART (LPUART4) driver intended
 * to replace PRINTF for high-rate streaming telemetry on the on-board
 * USB-CDC bridge.
 *
 * Wire protocol
 * -------------
 *
 *   [SYNC1=0xAA][SYNC2=0x55][SEQ][ID][LEN][PAYLOAD ... LEN bytes][CRC8]
 *
 *     - SYNC1, SYNC2 : fixed marker the host scans for to (re)acquire
 *                      frame alignment after a glitch.
 *     - SEQ          : 8-bit monotonic counter. The host can detect
 *                      drops by watching for skips ((seq - last) & 0xFF).
 *     - ID           : application-defined frame type byte.
 *     - LEN          : payload length in bytes (0..250).
 *     - PAYLOAD      : LEN application bytes.
 *     - CRC8         : CRC-8/SMBus (poly 0x07, init 0x00) computed over
 *                      [SEQ, ID, LEN, PAYLOAD]. Sync bytes are NOT
 *                      included so that they remain unique alignment
 *                      markers for the host parser.
 *
 * Host-side resync (analogous to rc_sync)
 * ---------------------------------------
 *
 *   1. Walk the byte stream looking for the SYNC1, SYNC2 pair.
 *   2. Read SEQ, ID, LEN, then LEN payload bytes, then CRC.
 *   3. Recompute CRC. On match deliver the frame; on mismatch advance
 *      the search window by ONE byte and retry. This lets the parser
 *      survive any single-byte loss / insertion -- on the next sync
 *      pair it re-acquires alignment, and from then on SEQ tells you
 *      how many frames were lost during the glitch.
 *
 * Author: Diego
 * Created: 2026-05-03
 * Revised: 2026-05-04 (resilient streaming rework)
 */

#ifndef DEBUG_UART_MCXN947_H_
#define DEBUG_UART_MCXN947_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Ring buffer size in bytes. MUST be a power of two. */
#ifndef DEBUG_UART_TX_BUF_SIZE
#define DEBUG_UART_TX_BUF_SIZE  2048U
#endif

/* Frame sync bytes used by debug_uart_send_frame(). */
#define DEBUG_UART_FRAME_SYNC1  0xAAU
#define DEBUG_UART_FRAME_SYNC2  0x55U

/* Frame overhead in bytes: SYNC1 + SYNC2 + SEQ + ID + LEN + CRC8. */
#define DEBUG_UART_FRAME_OVERHEAD  6U

/* Maximum payload length (so the whole frame fits in 256 bytes). */
#define DEBUG_UART_MAX_PAYLOAD     250U

/* Suggested frame IDs. The application is free to define more. */
#define DEBUG_UART_FRAME_ID_TELEMETRY  0x01U  /* per-motor telemetry */
#define DEBUG_UART_FRAME_ID_LOG        0x02U  /* free-form ASCII */
#define DEBUG_UART_FRAME_ID_RC         0x03U  /* RC channel snapshot */

/*!
 * @brief Initialise LPUART4 in TX-only, non-blocking ring-buffer mode.
 *
 * Clears any state left by the SDK debug console and takes ownership of
 * LPUART4. After calling this, do NOT use PRINTF (the SDK debug console
 * handle is no longer consistent with the hardware state).
 *
 * @param baudrate UART baud (e.g. 115200, 230400, 460800, 921600).
 *                 Higher is better -- 921600 leaves plenty of margin
 *                 for 500 Hz telemetry without backing the ring up.
 */
void debug_uart_init(uint32_t baudrate);

/*!
 * @brief Push raw bytes into the TX ring buffer. Never blocks.
 *
 * Safe to call from ANY context (task or ISR) once debug_uart_init()
 * has returned. If there isn't enough free space to fit the whole
 * payload, only what fits is queued and the rest is counted as drops.
 * Callers that require all-or-nothing semantics should check free
 * space first via debug_uart_free() or use debug_uart_send_frame().
 *
 * @return Number of bytes actually queued (0..len).
 */
size_t debug_uart_write(const uint8_t *data, size_t len);

/*!
 * @brief Convenience: push a NUL-terminated ASCII string.
 *
 * Safe from any context. Note: ASCII writes have NO frame markers, so
 * mixing them on the same wire as binary frames will desync the host
 * parser. Reserve this helper for one-shot startup messages before
 * the streaming begins.
 */
size_t debug_uart_print(const char *str);

/*!
 * @brief Send a binary frame using the protocol documented above.
 *
 * Safe to call from ANY context (task or ISR). The whole frame is
 * published atomically: if there isn't enough free space for the
 * complete frame, nothing is queued and the function returns false
 * (the entire frame is counted as a frame drop).
 *
 * The driver auto-increments SEQ on every successful publish.
 *
 * @param id      Frame ID byte (application-defined).
 * @param payload Pointer to payload bytes (may be NULL if len == 0).
 * @param len     Payload length in bytes (0 .. DEBUG_UART_MAX_PAYLOAD).
 * @return true if the frame was queued, false if the buffer is full,
 *         the driver is not initialised, or len is out of range.
 */
bool debug_uart_send_frame(uint8_t id, const void *payload, uint8_t len);

/*!
 * @brief Bytes currently waiting to be transmitted.
 */
size_t debug_uart_pending(void);

/*!
 * @brief Bytes currently free in the ring buffer.
 */
size_t debug_uart_free(void);

/*!
 * @brief Number of bytes that were dropped because the ring buffer
 *        was full. Useful as a health metric in long runs.
 */
uint32_t debug_uart_drops(void);

/*!
 * @brief Number of FRAMES that failed to publish (couldn't fit in the
 *        ring buffer). Each entry here also bumped debug_uart_drops()
 *        by the size of the would-be frame.
 */
uint32_t debug_uart_frame_drops(void);

/*!
 * @brief Last sequence number that was sent. Useful for sanity checks
 *        and for verifying that the host is in sync with the device.
 */
uint8_t debug_uart_last_seq(void);

#endif /* DEBUG_UART_MCXN947_H_ */
