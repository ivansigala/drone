/*
    bmp581.h
    Author: Diego
    Created on: 30, April 2026

    Driver for the Bosch BMP581 barometric pressure / temperature sensor.

    The public interface is intentionally laid out to mirror bme280.h so
    the kalman_z pipeline can swap sensors at the same call sites.  The
    main behavioural differences (handled inside bmp581.c):

      • The BMP581 returns *already-compensated* pressure/temperature
        values, so there is no calibration trim block to fetch.  The
        bme280_read_calibration() step has no equivalent here.

      • The output is exposed in the same fixed-point format as the
        BME280 driver, so kalman_z can stay agnostic:
            data.temperature : int32_t, units 0.01 °C
            data.pressure    : int64_t, units Q24.8 Pa  (÷256 → Pa)

      • SPI framing on this LPSPI bus matches the BME280: address byte,
        then N data bytes — no dummy phase.  See the note at the top of
        bmp581_reg.h for the empirical evidence.
 */


#ifndef BMP581_H_
#define BMP581_H_

#ifdef MCXN947
#include "spi_driver_mcxn947.h"
#include "gpio_driver_mcxn947.h"
#endif


typedef struct bmp581_data_s
{
    int32_t  temperature;   /* 0.01 °C units    — e.g. 5123 → 51.23 °C        */
    int64_t  pressure;      /* Q24.8 Pa units   — e.g. 24674867 → 96386.2 Pa  */
} bmp581_data_t;


typedef struct bmp581_ctrl_s
{
    spi_ctrl_t    spi_ctrl;
    /* gpio_event = BMP581 INT pin (data-ready).  Configured as input with
     * a falling-edge interrupt so the driver behaves the same way the
     * BNO085 HINT pin does: the gpio callback is invoked from the GPIO
     * port ISR every time a fresh sample lands in the shadow registers. */
    gpio_ctrl_t   gpio_event;
    bmp581_data_t data;
} bmp581_ctrl_t;


/*!
 * @brief Initialises the SPI peripheral, soft-resets the sensor, verifies
 *        the chip ID, writes the OSR / ODR / IIR / INT registers, and
 *        finally puts the BMP581 in continuous (NORMAL) mode.
 *
 *        After this returns successfully the BMP581 is producing one
 *        filtered pressure + temperature sample per ODR period
 *        (~50 Hz with the defaults in bmp581_reg.h) and the INT pin
 *        pulses active-low on every fresh sample.  @p gpio_callback is
 *        invoked from the GPIO port ISR on each falling edge — typical
 *        callback bodies clear the GPIO interrupt flag and then signal
 *        a FreeRTOS task with vTaskNotifyGiveFromISR().
 *
 *        The caller owns the bmp581_ctrl_t and is responsible for
 *        configuring bmp_ctl->gpio_event (gpio_base, port_base, pin) to
 *        the board pin connected to the BMP581 INT line BEFORE calling
 *        this function.  The driver fills spi_ctrl with the default
 *        barometer SPI configuration internally.
 *
 * @param  bmp_ctl       Pointer to a bmp581_ctrl_t the caller owns.
 * @param  gpio_callback ISR function pointer (void (*)(void)) registered
 *                       on the INT pin's falling edge.  Pass NULL to
 *                       skip GPIO configuration entirely (polling-only
 *                       use — not the recommended path).
 */
status_t bmp581_init(bmp581_ctrl_t *bmp_ctl, void *gpio_callback);


/*!
 * @brief Performs a 6-byte burst SPI read of the output registers
 *        (0x1D..0x22), sign-extends the 24-bit temperature, and converts
 *        both fields into the fixed-point units stored in bmp_ctl->data.
 *
 *        The BMP581 returns hardware-compensated values, so unlike the
 *        BME280 there is no software compensation step.  This function
 *        simply reformats the raw register data.
 */
void bmp581_parse_data(bmp581_ctrl_t *bmp_ctl);


#endif /* BMP581_H_ */
