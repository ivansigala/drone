/*
    vl53l0x.h
    Author: Diego
    Created on: 5, May 2026

    Driver for the ST VL53L0X Time-of-Flight ranging sensor on LPI2C2.

    The driver follows the same shape as the BMP581 / BNO085 ones in this
    project: a `*_ctrl_t` the caller owns, `*_init()` that brings the chip
    up and (optionally) attaches a falling-edge GPIO callback for the
    data-ready line, and a non-blocking read function that pulls the most
    recent distance into the ctrl block.

    Modes
    -----
    init() leaves the sensor in continuous back-to-back ranging mode (≈ 30
    samples/s with the default sequence config).  GPIO1 of the sensor is
    configured to assert active-low on every fresh sample — wire it to a
    GPIO input on the MCU, hand the falling-edge callback to init(), and
    you get an ISR-driven feed identical to how the BMP581 / BNO085 work.

    All I²C traffic goes through i2c_driver_mcxn947.c, which uses eDMA so
    the calling task sleeps on a semaphore for the duration of the bus
    transaction instead of busy-waiting.  Pass NULL for gpio_callback if
    you'd rather poll RESULT_INTERRUPT_STATUS in your task loop.
 */


#ifndef VL53L0X_H_
#define VL53L0X_H_

#include <stdint.h>
#include "fsl_common.h"

/* ---------------------------------------------------------------------------
 *  Calibration toggle
 *
 *  Define VL53L0X_PERFORM_CALIBRATION (uncomment the line below, or pass it
 *  as -DVL53L0X_PERFORM_CALIBRATION at compile time) to run the full
 *  ST-API-equivalent bring-up at every boot:
 *
 *    1. Read SPAD info from NVM (which detectors are good on this die).
 *    2. Apply the SPAD enable map.
 *    3. Load the ST "tuning settings" block (~80 register writes).
 *    4. Reference VHV calibration  (avalanche-bias high-voltage).
 *    5. Reference phase calibration (timing PLL lock).
 *
 *  Without these the chip can ack and emit interrupts, but real-world
 *  ranging fails with status = 6 / distance = 8190 mm because the phase
 *  isn't calibrated.  See vl53l0x.c file header for the full trade-off.
 *
 *  Cost: ~30–50 ms one-time at init() and ~150 extra lines of driver
 *  code linked in.  Leave it disabled if you only need to verify the
 *  I²C / IRQ pipeline.
 * ------------------------------------------------------------------------- */
#define VL53L0X_PERFORM_CALIBRATION 1

#ifdef MCXN947
#include "i2c_driver_mcxn947.h"
#include "gpio_driver_mcxn947.h"
#endif


typedef struct vl53l0x_data_s
{
    uint16_t distance_mm;     /* last raw range reading [mm], 8190 = out-of-range */
    uint8_t  range_status;    /* lower nibble of RESULT_RANGE_STATUS               */
} vl53l0x_data_t;


typedef struct vl53l0x_ctrl_s
{
    /* Hardware                                                             */
    i2c_ctrl_t   i2c_ctrl;
    /* Optional GPIO1 (data-ready) — open-drain on the sensor side.         *
     * Configured as input with falling-edge interrupt when init() is given *
     * a non-NULL gpio_callback.  Leave the .gpio_base / .pin / .port_base  *
     * fields zeroed when polling-only.                                     */
    gpio_ctrl_t  gpio_event;

    /* Configuration                                                        */
    uint8_t      i2c_addr;        /* 7-bit slave address (default 0x29)     */
    uint8_t      stop_variable;   /* captured during init, used elsewhere   */

    /* Latest decoded values                                                */
    vl53l0x_data_t data;
} vl53l0x_ctrl_t;


/*!
 * @brief  Brings the VL53L0X up: initialises LPI2C, verifies the model
 *         ID, runs the minimal Pololu-compatible init sequence, configures
 *         GPIO1 for "new-sample-ready" interrupts and starts continuous
 *         back-to-back ranging.
 *
 *         The caller owns the vl53l0x_ctrl_t and is responsible for
 *         setting i2c_addr (or leaving it 0 to use the 0x29 default) and
 *         populating gpio_event.{gpio_base, port_base, pin} BEFORE calling
 *         this function — same contract the BMP581 driver uses for its
 *         INT pin.
 *
 * @param  vl              Pointer to a caller-owned vl53l0x_ctrl_t.
 * @param  gpio_callback   Falling-edge ISR (void (*)(void)) attached to the
 *                         GPIO1 input.  Pass NULL to skip GPIO setup
 *                         entirely (polling-only mode).
 */
status_t vl53l0x_init(vl53l0x_ctrl_t *vl, void *gpio_callback);


/*!
 * @brief  Reads the latest distance from RESULT_RANGE_STATUS, decodes it
 *         into vl->data.distance_mm, and clears the GPIO1 interrupt so the
 *         next sample can fire.  All I²C traffic is non-blocking (DMA).
 *
 *         Call this from the FreeRTOS task that's woken by the GPIO1
 *         falling-edge ISR (or just on a vTaskDelay loop in polling mode).
 *
 * @param  vl   Initialised vl53l0x_ctrl_t.
 * @return kStatus_Success on a clean transaction; the underlying I²C
 *         status otherwise.
 */
status_t vl53l0x_read_distance(vl53l0x_ctrl_t *vl);


#endif /* VL53L0X_H_ */
