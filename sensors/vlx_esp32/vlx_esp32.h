/*
    vlx_esp32.h
    Author: Diego
    Created on: 18, May 2026

    Driver for an ESP32 SPI-slave bridge that publishes VL53L0X range data.

    Wire model
    ----------
    A second board (ESP32-C3) runs the actual VL53L0X over I2C, receives the
    distance over ESP-NOW, and exposes the most recent sample as an SPI slave.
    Every time this MCU clocks a transaction the ESP32 returns the latest
    `vlx_distance_msg_t` (16 bytes) on MISO; the master payload is ignored
    by the slave.

    Bus / pin reuse
    ---------------
    The link runs on LPSPI2 with the same pin set that previously talked to
    the BME280 (PORT4 pins 0..3) — only the clock polarity/phase changes
    (Mode 0 instead of Mode 3) to match the ESP32 slave.
 */

#ifndef VLX_ESP32_H_
#define VLX_ESP32_H_

#include <stdint.h>
#include "fsl_common.h"

#ifdef MCXN947
#include "spi_driver_mcxn947.h"
#endif

/* Magic word the ESP32 stamps into every published message.  Used here to
 * sanity-check that the bytes we clocked off MISO are actually a fresh
 * frame and not bus noise / a stale FIFO read.                              */
#define VLX_MSG_MAGIC          0x58534C56U   /* "VLSX" little-endian */

/* Length in bytes of a single SPI exchange.  Matches the ESP32's
 * SPI_PAYLOAD_SIZE.                                                         */
#define VLX_PAYLOAD_SIZE       16U


typedef enum
{
    VLX_STATUS_OK            = 0,
    VLX_STATUS_TIMEOUT       = 1,
    VLX_STATUS_I2C_ERROR     = 2,
    VLX_STATUS_OUT_OF_RANGE  = 3
} vlx_status_t;


/* Mirror of the struct the ESP32 sends.  Keep packed + identical layout so
 * a flat memcpy from the SPI RX buffer reconstructs every field correctly
 * (both MCUs are little-endian, so byte order is preserved).                */
typedef struct __attribute__((packed)) vlx_distance_msg_s
{
    uint32_t magic;          /* must equal VLX_MSG_MAGIC                     */
    uint32_t seq;            /* monotonically increasing sample counter      */
    uint32_t timestamp_ms;   /* ESP32 millis() at capture time               */
    uint16_t distance_mm;    /* compensated VL53L0X range in millimetres     */
    uint8_t  status;         /* one of vlx_status_t                          */
    uint8_t  reserved;       /* padding — present on the wire                */
} vlx_distance_msg_t;

/* Compile-time guarantee that the on-wire layout matches what we declared. */
typedef char vlx_msg_size_check_t[(sizeof(vlx_distance_msg_t) == VLX_PAYLOAD_SIZE) ? 1 : -1];


typedef struct vlx_ctrl_s
{
    spi_ctrl_t          spi_ctrl;  /* bus configuration                      */
    vlx_distance_msg_t  data;      /* last successfully decoded sample       */
} vlx_ctrl_t;


/*!
 * @brief Initialises the SPI master used to talk to the ESP32 SPI slave.
 *
 *        Fills @p vlx->spi_ctrl with the VLX defaults (LPSPI2, Mode 0, DMA)
 *        and brings the LPSPI peripheral up.  No traffic is generated yet;
 *        the ESP32 only sends data when we clock it.
 *
 * @param vlx Pointer to a vlx_ctrl_t owned by the caller.
 * @return    kStatus_Success on success, an LPSPI error code otherwise.
 */
status_t vlx_init(vlx_ctrl_t *vlx);


/*!
 * @brief Clocks a full 16-byte transaction, copies the response into
 *        @p vlx->data and validates the magic word.
 *
 *        On success the most recent sample is available in vlx->data.
 *        On a corrupted/incomplete frame the function returns kStatus_Fail
 *        and leaves vlx->data untouched so callers can keep printing the
 *        last known good value.
 *
 * @param vlx Pointer to a previously initialised vlx_ctrl_t.
 * @return    kStatus_Success on a valid frame, kStatus_Fail on bad magic,
 *            or an LPSPI error code on a failed transfer.
 */
status_t vlx_read(vlx_ctrl_t *vlx);


#endif /* VLX_ESP32_H_ */
