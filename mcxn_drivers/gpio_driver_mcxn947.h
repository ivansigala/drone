/*!
 * @file gpio_driver_mcxn947.h
 * @brief GPIO driver for MCXN947 microcontroller
 * 
 * This header provides high-level API for GPIO management on the NXP MCXN947 MCU including:
 * - GPIO pin configuration (direction, pull resistors, drive strength)
 * - GPIO output control (set, toggle)
 * - GPIO interrupt handling (attach callbacks, ISR handlers)
 * 
 * @author Diego
 * @date April 10, 2026
 * 
 * @details
 * The driver provides a simple interface using the gpio_ctrl_t structure that contains
 * all necessary GPIO configuration. Key functions:
 * - gpio_init(): Initialize a GPIO pin with predefined settings
 * - gpio_attach_interrupt(): Attach interrupt handler to GPIO pin
 * - gpio_set_output(): Set GPIO output value
 * - gpio_toggle_output(): Toggle GPIO output state
 * 
 * @note This driver uses callback-based interrupt handling. Each GPIO port has dedicated
 *       ISR handlers (GPIO00_IRQHandler through GPIO51_IRQHandler) that invoke the
 *       registered callbacks.
 * 
 * @example
 * @code
 * // Initialize GPIO pin as input with interrupt
 * gpio_ctrl_t imu_int = {
 *     .gpio_base = GPIO1,
 *     .port_base = PORT1,
 *     .pin = 17,
 *     .dir = gpio_input
 * };
 * 
 * // Initialize the pin
 * gpio_init(&imu_int);
 * 
 * // Attach interrupt handler
 * void my_callback(void) {
 *     // Handle interrupt event
 * }
 * gpio_attach_interrupt(&imu_int, my_callback);
 * @endcode
 */

#ifndef GPIO_DRIVER_MCXN947_H_
#define GPIO_DRIVER_MCXN947_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"


/*! @brief GPIO pin direction enumeration */
typedef enum gpio_direction_s{
    gpio_input = 0,  /*!< Configure pin as input */
    gpio_output,     /*!< Configure pin as output */
} gpio_direction_t;


/*! 
 * @brief GPIO control structure containing all configuration for a GPIO pin
 * 
 * This structure holds the necessary information to configure and control a GPIO pin.
 * It should be populated before passing to gpio_init().
 */
typedef struct gpio_ctrl_s{
    GPIO_Type *gpio_base;      /*!< GPIO peripheral base address (e.g., GPIO1) */
    PORT_Type *port_base;      /*!< PORT peripheral base address (e.g., PORT1) */
    gpio_direction_t dir;      /*!< Pin direction: gpio_input or gpio_output */
    uint8_t pin;               /*!< Pin number (0-31) */
} gpio_ctrl_t;

/*!
 * @name GPIO Initialization Functions
 * @{
 */

/*!
 * @brief Initialize a GPIO pin with settings from gpio_ctrl_t structure
 * @param gpio Pointer to gpio_ctrl_t structure with configuration
 */
void gpio_init(gpio_ctrl_t *gpio);

/*!
 * @brief Low-level PORT and GPIO module setup
 * @param gpio_base GPIO peripheral base address
 * @param port_base PORT peripheral base address
 * @param pin Pin number (0-31)
 */
void gpio_setup(GPIO_Type* gpio_base, PORT_Type *port_base, uint8_t pin);

/*!
 * @brief Set GPIO pin direction (input or output)
 * @param gpio_base GPIO peripheral base address
 * @param pin Pin number (0-31)
 * @param direction gpio_input or gpio_output
 */
void gpio_set_direction(GPIO_Type* gpio_base, uint8_t pin, gpio_direction_t direction);

/** @} */

/*!
 * @name GPIO Interrupt Functions
 * @{
 */

/*!
 * @brief Attach interrupt handler and enable interrupt on GPIO pin
 * @param gpio Pointer to gpio_ctrl_t structure
 * @param callback Callback function pointer (void (*)(void))
 */
void gpio_attach_interrupt(gpio_ctrl_t *gpio, void* callback);

/*!
 * @brief Clear interrupt flag for a GPIO pin
 * @param gpio_base GPIO peripheral base address
 * @param pin Pin number (0-31)
 */
void gpio_clear_interrupt_flag(GPIO_Type* gpio_base, uint8_t pin);


/*!
 * @brief Set output value for a GPIO pin
 * @param gpio Pointer to gpio_ctrl_t structure
 * @param val Output value (0 or 1)
 */
void gpio_set_output(gpio_ctrl_t *gpio, uint32_t val);

/*!
 * @brief Read input value from a GPIO pin
 * @param gpio Pointer to gpio_ctrl_t structure
 * @return Input value (0 or 1)
 */
uint32_t gpio_read_input(gpio_ctrl_t *gpio);

/*!
 * @brief Toggle output state of a GPIO pin
 * @param gpio Pointer to gpio_ctrl_t structure
 */
void gpio_toggle_output(gpio_ctrl_t *gpio);


#endif /* GPIO_DRIVER_MCXN947_H_ */
