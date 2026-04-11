/*
    gpio_driver_mcxn947.c
    Author: Diego
    Created on: 10, April 2026
 */

#include "gpio_driver_mcxn947.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/
/*!
 * @brief Handler function pointers for GPIO port interrupts.
 * 
 * These static pointers hold the callback functions for each GPIO IRQ handler.
 * They are assigned when gpio_attach_interrupt() is called and invoked within
 * the IRQ handlers (GPIOnn_IRQHandler).
 * - GPIO0X_HANDLER: Handlers for GPIO port 0 pins
 * - GPIO1X_HANDLER: Handlers for GPIO port 1 pins
 * - GPIO2X_HANDLER: Handlers for GPIO port 2 pins
 * - GPIO3X_HANDLER: Handlers for GPIO port 3 pins
 * - GPIO4X_HANDLER: Handlers for GPIO port 4 pins
 * - GPIO5X_HANDLER: Handlers for GPIO port 5 pins
 */
void (*GPIO00_HANDLER)(void);
void (*GPIO01_HANDLER)(void);
void (*GPIO10_HANDLER)(void);
void (*GPIO11_HANDLER)(void);
void (*GPIO20_HANDLER)(void);
void (*GPIO21_HANDLER)(void);
void (*GPIO30_HANDLER)(void);
void (*GPIO31_HANDLER)(void);
void (*GPIO40_HANDLER)(void);
void (*GPIO41_HANDLER)(void);
void (*GPIO50_HANDLER)(void);
void (*GPIO51_HANDLER)(void);


/*******************************************************************************
 * Functions
 ******************************************************************************/

/*!
 * @brief Sets the output value of a GPIO pin.
 * 
 * Writes a logic level (0 or 1) to the specified GPIO pin.
 * The pin must be configured as an output using gpio_init() before calling this function.
 * 
 * @param gpio Pointer to the GPIO control structure containing base address and pin number.
 * @param val The output value to set: 0 for logic low, 1 for logic high.
 * 
 * @return void
 */
void gpio_set_output(gpio_ctrl_t *gpio, uint32_t val)
{
    GPIO_PinWrite(gpio->gpio_base, gpio->pin, val);
}

/*!
 * @brief Reads the current input of a GPIO pin.
 *
 * @param gpio Pointer to the GPIO control structure containing base address and pin number.
 * @return The input value: 0 for logic low, 1 for logic high.
 */
uint32_t gpio_read_input(gpio_ctrl_t *gpio){
    return GPIO_PinRead(gpio->gpio_base, gpio->pin);
}

/*!
 * @brief Toggles the output state of a GPIO pin.
 * 
 * Inverts the current output logic level of the specified GPIO pin.
 * If the pin is high, it will be set low. If low, it will be set high.
 * The pin must be configured as an output using gpio_init() before calling this function.
 * 
 * @param gpio Pointer to the GPIO control structure containing base address and pin number.
 * 
 * @return void
 */
void gpio_toggle_output(gpio_ctrl_t *gpio)
{
    GPIO_PortToggle(gpio->gpio_base, 1U << gpio->pin);
}

/*!
 * @brief Initializes a GPIO pin with the settings specified in the gpio_ctrl_t structure.
 * 
 * This is the main GPIO initialization function. It performs pin configuration including:
 * - Port and GPIO clock enabling
 * - Pin multiplexing and electrical configuration (pull resistors, slew rate, etc.)
 * - GPIO direction setting (input or output)
 * 
 * Must be called before using the GPIO pin for input/output operations.
 * 
 * @param gpio Pointer to a gpio_ctrl_t structure containing:
 *             - gpio_base: GPIO peripheral base address
 *             - port_base: PORT peripheral base address
 *             - pin: Pin number (0-31)
 *             - dir: Pin direction (gpio_input or gpio_output)
 * 
 * @return void
 * 
 * @note The gpio_ctrl_t structure must be fully initialized before calling this function.
 */
void gpio_init(gpio_ctrl_t *gpio)
{
    gpio_setup(gpio->gpio_base, gpio->port_base, gpio->pin);
    gpio_set_direction(gpio->gpio_base, gpio->pin, gpio->dir);

    return;

}

/*!
 * @brief Sets the direction (input or output) for a GPIO pin.
 * 
 * Configures whether the specified GPIO pin operates as an input or output
 * by modifying the Port Data Direction Register (PDDR).
 * 
 * @param gpio_base Pointer to the GPIO peripheral base address (e.g., GPIO1).
 * @param pin Pin number (0-31) within the GPIO port.
 * @param direction Direction to set: gpio_input (0) or gpio_output (1).
 * 
 * @return void
 * 
 * @note gpio_setup() should typically be called before this function to configure
 *       the port and GPIO module.
 */
void gpio_set_direction(GPIO_Type* gpio_base, uint8_t pin, gpio_direction_t direction)
{

    if (direction == gpio_input)
    {
        gpio_base->PDDR &= GPIO_FIT_REG(~(1UL << pin));
    }
    else
    {
        gpio_base->PDDR |= GPIO_FIT_REG((1UL << pin));
    }

    return;
}

/*!
 * @brief Configures the PORT and GPIO module settings for a pin.
 * 
 * Low-level hardware configuration that enables clocks, sets pin multiplexing,
 * and configures electrical characteristics (pull resistors, slew rate, etc.).
 * This function is typically called by gpio_init().
 * 
 * Configuration applied:
 * - Port Input Enable (reading pad state)
 * - Pull-down resistor enabled
 * - Fast slew rate
 * - Low drive strength
 * - GPIO mux mode
 * - Input buffer enabled
 * 
 * @param gpio_base Pointer to the GPIO peripheral base address (e.g., GPIO1).
 * @param port_base Pointer to the PORT peripheral base address (e.g., PORT1).
 * @param pin Pin number (0-31) within the port.
 * 
 * @return void
 * 
 * @note This function automatically enables both PORT and GPIO clocks for the
 *       corresponding port (PORT0/GPIO0 through PORT5/GPIO5).
 */
void gpio_setup(GPIO_Type* gpio_base, PORT_Type *port_base, uint8_t pin)
{
    gpio_pin_config_t pinConfig = {
        kGPIO_DigitalInput,
        0,
    };

    const port_pin_config_t port_config = {
        kPORT_PullDown,             // Pull-down for your 3.3V wire test
        kPORT_LowPullResistor,
        kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable,
        kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength,
        kPORT_MuxAsGpio,           
        kPORT_InputBufferEnable,    
        kPORT_InputNormal,
        kPORT_UnlockRegister
    };

    /* Enable GPIO clock if not already enabled */
    if(port_base == PORT0) {
        CLOCK_EnableClock(kCLOCK_Port0);
        CLOCK_EnableClock(kCLOCK_Gpio0);
    } else if(port_base == PORT1) {
        CLOCK_EnableClock(kCLOCK_Port1);
        CLOCK_EnableClock(kCLOCK_Gpio1);
    } else if(port_base == PORT2) {
        CLOCK_EnableClock(kCLOCK_Port2);
        CLOCK_EnableClock(kCLOCK_Gpio2);
    } else if(port_base == PORT3) {
        CLOCK_EnableClock(kCLOCK_Port3);
        CLOCK_EnableClock(kCLOCK_Gpio3);
    } else if(port_base == PORT4) {
        CLOCK_EnableClock(kCLOCK_Port4);
        CLOCK_EnableClock(kCLOCK_Gpio4);
    } 
    
    PORT_SetPinConfig(port_base, pin, &port_config);

    GPIO_PinInit(gpio_base, pin, &pinConfig);
}

/*!
 * @brief Attaches an interrupt handler to a GPIO pin.
 */
void gpio_attach_interrupt(gpio_ctrl_t *gpio, void* callback){
    
    /* Then configure GPIO interrupt output */
    GPIO_SetPinInterruptConfig(gpio->gpio_base, gpio->pin, kGPIO_InterruptFallingEdge);

    if(gpio->gpio_base == GPIO0){
        EnableIRQ(GPIO00_IRQn);
        EnableIRQ(GPIO01_IRQn);
        GPIO00_HANDLER = callback;
        GPIO01_HANDLER = callback;
    } else if(gpio->gpio_base == GPIO1){
        EnableIRQ(GPIO10_IRQn);
        EnableIRQ(GPIO11_IRQn);
        GPIO10_HANDLER = callback;
        GPIO11_HANDLER = callback;
    }else if(gpio->gpio_base == GPIO2){
        EnableIRQ(GPIO20_IRQn);
        EnableIRQ(GPIO21_IRQn);
        GPIO20_HANDLER = callback;
        GPIO21_HANDLER = callback;
    } else if(gpio->gpio_base == GPIO3){
        EnableIRQ(GPIO30_IRQn);
        EnableIRQ(GPIO31_IRQn);
        GPIO30_HANDLER = callback;
        GPIO31_HANDLER = callback;
    } else if(gpio->gpio_base == GPIO4){
        EnableIRQ(GPIO40_IRQn);
        EnableIRQ(GPIO41_IRQn);
        GPIO40_HANDLER = callback;
        GPIO41_HANDLER = callback;
    } else if(gpio->gpio_base == GPIO5){
        EnableIRQ(GPIO50_IRQn);
        EnableIRQ(GPIO51_IRQn);
        GPIO50_HANDLER = callback;
        GPIO51_HANDLER = callback;
    } else {
        return;
    }

    return;

}

/*!
 * @brief Clears the interrupt flag for a specific GPIO pin.
 * 
 * Clears the interrupt pending flag(s) for the specified GPIO pin in the GPIO module.
 * This is typically called at the beginning of an ISR to acknowledge the interrupt event.
 * 
 * @param gpio_base Pointer to the GPIO peripheral base address (e.g., GPIO1).
 * @param pin Pin number (0-31) whose interrupt flag should be cleared.
 * 
 * @return void
 * 
 * @note This function must be called within the interrupt handler to prevent
 *       the interrupt from re-triggering immediately.
 */
void gpio_clear_interrupt_flag(GPIO_Type* gpio_base, uint8_t pin){
	
    __disable_irq();
    GPIO_GpioClearInterruptFlags(gpio_base, 0x1U << pin);
    __enable_irq();
}


/*!
 * @brief Interrupt handler for GPIO port 0, pin 0.
 * 
 * Executes the callback function registered via gpio_attach_interrupt() when a GPIO0.0 interrupt occurs.
 * This is automatically called by the NVIC on GPIO0.0 interrupt events.
 * 
 * @return void
 */
void GPIO00_IRQHandler(void){
    if(GPIO00_HANDLER != NULL){
        GPIO00_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Interrupt handler for GPIO port 0, pin 1.
 * 
 * Executes the callback function registered via gpio_attach_interrupt() when a GPIO0.1 interrupt occurs.
 * This is automatically called by the NVIC on GPIO0.1 interrupt events.
 * 
 * @return void
 */
void GPIO01_IRQHandler(void){
    if(GPIO01_HANDLER != NULL){
        GPIO01_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Interrupt handler for GPIO port 1, pin 0.
 * 
 * Executes the callback function registered via gpio_attach_interrupt() when a GPIO1.0 interrupt occurs.
 * This is automatically called by the NVIC on GPIO1.0 interrupt events.
 * 
 * @return void
 */
void GPIO10_IRQHandler(void){
    if(GPIO10_HANDLER != NULL){
        GPIO10_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Interrupt handler for GPIO port 1, pin 1.
 * 
 * Executes the callback function registered via gpio_attach_interrupt() when a GPIO1.1 interrupt occurs.
 * This is automatically called by the NVIC on GPIO1.1 interrupt events.
 * 
 * @return void
 */
void GPIO11_IRQHandler(void){
    if(GPIO11_HANDLER != NULL){
        GPIO11_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Interrupt handler for GPIO port 2, pin 0.
 * 
 * Executes the callback function registered via gpio_attach_interrupt() when a GPIO2.0 interrupt occurs.
 * This is automatically called by the NVIC on GPIO2.0 interrupt events.
 * 
 * @return void
 */
void GPIO20_IRQHandler(void){
    if(GPIO20_HANDLER != NULL){
        GPIO20_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Interrupt handler for GPIO port 2, pin 1.
 * 
 * Executes the callback function registered via gpio_attach_interrupt() when a GPIO2.1 interrupt occurs.
 * This is automatically called by the NVIC on GPIO2.1 interrupt events.
 * 
 * @return void
 */
void GPIO21_IRQHandler(void){
    if(GPIO21_HANDLER != NULL){
        GPIO21_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Interrupt handler for GPIO port 3, pin 0.
 * 
 * Executes the callback function registered via gpio_attach_interrupt() when a GPIO3.0 interrupt occurs.
 * This is automatically called by the NVIC on GPIO3.0 interrupt events.
 * 
 * @return void
 */
void GPIO30_IRQHandler(void){
    if(GPIO30_HANDLER != NULL){
        GPIO30_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Interrupt handler for GPIO port 3, pin 1.
 * 
 * Executes the callback function registered via gpio_attach_interrupt() when a GPIO3.1 interrupt occurs.
 * This is automatically called by the NVIC on GPIO3.1 interrupt events.
 * 
 * @return void
 */
void GPIO31_IRQHandler(void){
    if(GPIO31_HANDLER != NULL){
        GPIO31_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Interrupt handler for GPIO port 4, pin 0.
 * 
 * Executes the callback function registered via gpio_attach_interrupt() when a GPIO4.0 interrupt occurs.
 * This is automatically called by the NVIC on GPIO4.0 interrupt events.
 * 
 * @return void
 */
void GPIO40_IRQHandler(void){

    if(GPIO40_HANDLER != NULL){
        GPIO40_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Interrupt handler for GPIO port 4, pin 1.
 * 
 * Executes the callback function registered via gpio_attach_interrupt() when a GPIO4.1 interrupt occurs.
 * This is automatically called by the NVIC on GPIO4.1 interrupt events.
 * 
 * @return void
 */
void GPIO41_IRQHandler(void){
    if(GPIO41_HANDLER != NULL){
        GPIO41_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Interrupt handler for GPIO port 5, pin 0.
 * 
 * Executes the callback function registered via gpio_attach_interrupt() when a GPIO5.0 interrupt occurs.
 * This is automatically called by the NVIC on GPIO5.0 interrupt events.
 * 
 * @return void
 */
void GPIO50_IRQHandler(void){
    if(GPIO50_HANDLER != NULL){
        GPIO50_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Interrupt handler for GPIO port 5, pin 1.
 * 
 * Executes the callback function registered via gpio_attach_interrupt() when a GPIO5.1 interrupt occurs.
 * This is automatically called by the NVIC on GPIO5.1 interrupt events.
 * 
 * @return void
 */
void GPIO51_IRQHandler(void){
    if(GPIO51_HANDLER != NULL){
        GPIO51_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}