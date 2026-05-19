/*
    vlx_esp32.c
    Author: Diego
    Created on: 18, May 2026

    SPI-master side of the ESP32 VLX bridge.  See vlx_esp32.h for the
    architecture description.

    Framing
    -------
    Every transaction is a fixed 16-byte burst:

        MOSI : 16 dummy bytes (0x00) — the ESP32 ignores the payload.
        MISO : the freshest vlx_distance_msg_t the ESP32 has staged.

    The LPSPI driver in this project is set up with kLPSPI_MasterByteSwap +
    kLPSPI_MasterPcsContinuous, which means PCS0 stays asserted for the
    whole 16-byte frame and bytes appear on the wire in array-index order.
    That matches the ESP32 slave's expectation of a single contiguous frame.

    Validation
    ----------
    Bad or stale reads (CS glitch, ESP32 not booted yet, etc.) tend to come
    back as 0xFF/0x00 patterns.  Filtering them by the magic word means
    callers always see either a fresh sample or the previous good value.
 */

#include "vlx_esp32.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "fsl_debug_console.h"


/* DMA-friendly scratch buffers.  Have to live in non-cacheable memory and
 * be word-aligned because the LPSPI eDMA path operates on them directly —
 * cached copies would be invisible to the DMA engine.                       */
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_vlx_tx[VLX_PAYLOAD_SIZE], 4);
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_vlx_rx[VLX_PAYLOAD_SIZE], 4);


status_t vlx_init(vlx_ctrl_t *vlx)
{
    if (vlx == NULL)
    {
        return kStatus_InvalidArgument;
    }

#ifdef MCXN947
    /* Same hardware as the BME280 (LPSPI2 / PORT4 pins 0..3 / DMA ch 9-10),
     * but with CPOL/CPHA flipped to Mode 0 so we talk to the ESP32 slave. */
    spi_get_defaultconfig_vlx(&vlx->spi_ctrl);
#endif

    /* Match the priority used by the BME280 driver so the eDMA completion
     * IRQ has the same scheduling latency as before.                        */
    NVIC_SetPriority(EDMA_0_CH9_IRQn,  5);
    NVIC_SetPriority(EDMA_0_CH10_IRQn, 5);

    /* Pre-populate vlx->data with a "no data yet" frame so callers don't
     * print uninitialised stack on the very first iteration before the
     * ESP32 has had a chance to publish anything.                           */
    memset(&vlx->data, 0, sizeof(vlx->data));
    vlx->data.magic  = VLX_MSG_MAGIC;
    vlx->data.status = (uint8_t)VLX_STATUS_TIMEOUT;

    return spi_init(&vlx->spi_ctrl);
}


status_t vlx_read(vlx_ctrl_t *vlx)
{
    if (vlx == NULL)
    {
        return kStatus_InvalidArgument;
    }

    /* The ESP32 ignores everything we send, but the bus still needs a
     * driven MOSI line — fill the TX buffer with a known pattern so the
     * line isn't left floating between transactions.                       */
    memset(s_vlx_tx, 0x00, sizeof(s_vlx_tx));
    memset(s_vlx_rx, 0x00, sizeof(s_vlx_rx));

    status_t status = spi_master_transfer(&vlx->spi_ctrl,
                                          s_vlx_tx,
                                          s_vlx_rx,
                                          VLX_PAYLOAD_SIZE);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Drop frames that don't carry the right magic.  This catches:
     *   - the bus coming up before the ESP32 finished booting
     *   - the ESP32 momentarily missing the SPI frame
     *   - the master clocking a transaction while no slave is attached
     * Leaving vlx->data alone means callers keep the last good value.       */
    vlx_distance_msg_t candidate;
    memcpy(&candidate, s_vlx_rx, sizeof(candidate));

    if (candidate.magic != VLX_MSG_MAGIC)
    {
        return kStatus_Fail;
    }

    vlx->data = candidate;
    return kStatus_Success;
}
