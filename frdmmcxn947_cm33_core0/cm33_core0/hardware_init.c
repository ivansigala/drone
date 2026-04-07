/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*${header:start}*/
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_clock.h"
/*${header:end}*/

/*${variable:start}*/

/*${variable:end}*/
/*${function:start}*/
void BOARD_InitHardware(void)
{
    /* attach FRO 12M to FLEXCOMM4 (debug console) */
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom4Clk, 1u);
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    CLOCK_SetClkDiv(kCLOCK_DivFlexcom1Clk, 1u);
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM1);

    CLOCK_SetClkDiv(kCLOCK_DivFlexcom7Clk, 1u);
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM7);

    CLOCK_EnableClock(kCLOCK_Dma0);

    BOARD_InitPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    /* Enable PWM1 SUB Clockn */
    SYSCON->PWM1SUBCTL |=
       (SYSCON_PWM1SUBCTL_CLK0_EN_MASK | SYSCON_PWM1SUBCTL_CLK1_EN_MASK | SYSCON_PWM1SUBCTL_CLK2_EN_MASK | SYSCON_PWM1SUBCTL_CLK3_EN_MASK);

}
/*${function:end}*/
