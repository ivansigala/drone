#include "gpio_driver_mcxn947.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/
//Pointer for handler assignment
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

void gpio_set_output(gpio_ctrl_t *gpio, uint32_t val)
{
    GPIO_PinWrite(gpio->gpio_base, gpio->pin, val);
}

void gpio_toggle_output(gpio_ctrl_t *gpio)
{
    GPIO_PortToggle(gpio->gpio_base, 1U << gpio->pin);
}

void gpio_init(gpio_ctrl_t *gpio)
{
    gpio_setup(gpio->gpio_base, gpio->port_base, gpio->pin);
    gpio_set_direction(gpio->gpio_base, gpio->pin, gpio->dir);

    return;

}

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

void gpio_attach_interrupt(gpio_ctrl_t *gpio, void* callback){
    
    /* Then configure GPIO interrupt output */
    GPIO_SetPinInterruptConfig(gpio->gpio_base, gpio->pin, kGPIO_InterruptRisingEdge);

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

void gpio_clear_interrupt_flag(GPIO_Type* gpio_base){
	// Clear interrupt flags
    GPIO_GpioClearInterruptFlags(gpio_base, 0xFFFFFFFF);
    
}


void GPIO00_IRQHandler(void){
    gpio_clear_interrupt_flag(GPIO0);
    if(GPIO00_HANDLER != NULL){
        GPIO00_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

void GPIO01_IRQHandler(void){
    gpio_clear_interrupt_flag(GPIO0);
    if(GPIO01_HANDLER != NULL){
        GPIO01_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

void GPIO10_IRQHandler(void){
    gpio_clear_interrupt_flag(GPIO1);
    if(GPIO10_HANDLER != NULL){
        GPIO10_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

void GPIO11_IRQHandler(void){
    gpio_clear_interrupt_flag(GPIO1);
    if(GPIO11_HANDLER != NULL){
        GPIO11_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

void GPIO20_IRQHandler(void){
    gpio_clear_interrupt_flag(GPIO2);
    if(GPIO20_HANDLER != NULL){
        GPIO20_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

void GPIO21_IRQHandler(void){
    gpio_clear_interrupt_flag(GPIO2);
    if(GPIO21_HANDLER != NULL){
        GPIO21_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

void GPIO30_IRQHandler(void){
    gpio_clear_interrupt_flag(GPIO3);
    if(GPIO30_HANDLER != NULL){
        GPIO30_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

void GPIO31_IRQHandler(void){
    gpio_clear_interrupt_flag(GPIO3);
    if(GPIO31_HANDLER != NULL){
        GPIO31_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

void GPIO40_IRQHandler(void){
    gpio_clear_interrupt_flag(GPIO4);

    if(GPIO40_HANDLER != NULL){
        GPIO40_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

void GPIO41_IRQHandler(void){
    gpio_clear_interrupt_flag(GPIO4);
    if(GPIO41_HANDLER != NULL){
        GPIO41_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

void GPIO50_IRQHandler(void){
    gpio_clear_interrupt_flag(GPIO5);
    if(GPIO50_HANDLER != NULL){
        GPIO50_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}

void GPIO51_IRQHandler(void){
    gpio_clear_interrupt_flag(GPIO5);
    if(GPIO51_HANDLER != NULL){
        GPIO51_HANDLER();
    }
    SDK_ISR_EXIT_BARRIER;
}