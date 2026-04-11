/*
 * GPIO_DRIVER.h
 *
 *  Created on: Sep 29, 2025
 *      Author: diego
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


typedef enum gpio_direction_s{
    gpio_input = 0,
    gpio_output,
} gpio_direction_t;


typedef struct gpio_ctrl_s{
    GPIO_Type *gpio_base;
    PORT_Type *port_base;
    gpio_direction_t dir;
    uint8_t pin;
} gpio_ctrl_t;

void gpio_init(gpio_ctrl_t *gpio);
void gpio_setup(GPIO_Type* gpio_base, PORT_Type *port_base, uint8_t pin);
void gpio_set_direction(GPIO_Type* gpio_base, uint8_t pin, gpio_direction_t direction);

void gpio_attach_interrupt(gpio_ctrl_t *gpio, void* callback);
void gpio_clear_interrupt_flag(GPIO_Type* gpio_base);


#endif /* GPIO_DRIVER_MCXN947_H_ */
