//*****************************************************************************
//
// pwm_dead_band.c - Example demonstrating the PWM dead-band generator.
//
// Copyright (c) 2019-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.2.0.295 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
//! \addtogroup pwm_examples_list
//! <h1>PWM Dead-band Generator Demo (pwm_dead_band)</h1>
//!
//! The example configures the PWM0 block to generate a 25% duty cycle signal
//! on PF2 with dead-band generation.  This will produce a complement of PF2 on
//! PF3 (75% duty cycle).  The dead-band generator is set to have a 10us delay
//! on the rising and falling edges of the PF2 PWM signal.
//!
//! This example uses the following peripherals and I/O signals.
//! - GPIO Port F peripheral (for PWM pins)
//! - M0PWM2 - PF2
//! - M0PWM3 - PF3
//!
//! UART0, connected to the Virtual Serial Port and running at 115,200, 8-N-1,
//! is used to display messages from this application.
//
//*****************************************************************************

//*****************************************************************************
//
// The variable g_ui32SysClock contains the system clock frequency in Hz.
//
//*****************************************************************************
uint32_t g_ui32SysClock;
uint32_t g_ui32PWMIncrement;
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void run_motor_speed(int level);
//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

//*****************************************************************************
//
// Configure PWM for dead-band generation.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32PWMClockRate;

    //
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240), 120000000);

    //
    // Initialize the UART.
    //
    ConfigureUART();


    //
    // The PWM peripheral must be enabled for use.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    //
    // Enable the GPIO port that is used for the PWM output.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Configure the GPIO pad for PWM function on pins PF2 and PF3.
    //
    MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);
    MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Set the PWM clock to be SysClk / 8.
    //
    MAP_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);

    // PWM generator period.
    //120Mhz ,15Mhz PWM
    ui32PWMClockRate = g_ui32SysClock / 8;
    //6Khz increment
    g_ui32PWMIncrement = ((ui32PWMClockRate / 250) / 10); //10% increment

    //
    // Configure PWM2 to count up/down without synchronization.
    // Note: Enabling the dead-band generator automatically couples the 2
    // outputs from the PWM block so we don't use the PWM synchronization.
    //
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                        PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    //
    // Set the PWM period to 250Hz.  To calculate the appropriate parameter
    // use the following equation: N = (1 / f) * PWMClk.  Where N is the
    // function parameter, f is the desired frequency, and PWMClk is the
    // PWM clock frequency based on the system clock.
    //
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, (ui32PWMClockRate / 250));

    //
    // Set PWM2 to a duty cycle of 25%.  You set the duty cycle as a function
    // of the period.  Since the period was set above, you can use the
    // PWMGenPeriodGet() function.  For this example the PWM will be high for
    // 25% of the time or (PWM Period / 4).
    //
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                         MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) / 4);

    //
    //MAP_PWMDeadBandEnable(PWM0_BASE, PWM_GEN_1, 150, 150);

    //
    // Enable the PWM Out Bit 2 (PF2) and Bit 3 (PF3) output signals.
    //
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);

    //
    // Enable the PWM generator block.
    //
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    //
    // Loop forever blinking an LED while the PWM signals are generated.
    //
    while(1)
    {
        //Making the motor run from 50 to 80 percent
        UARTprintf("setting motor speed 5x\n");
        run_motor_speed(5);

        MAP_SysCtlDelay(g_ui32SysClock / 3 );
        MAP_SysCtlDelay(g_ui32SysClock / 3 );

        UARTprintf("setting motor speed 6x\n");
        run_motor_speed(6);

        MAP_SysCtlDelay(g_ui32SysClock / 3 );
        MAP_SysCtlDelay(g_ui32SysClock / 3 );

        UARTprintf("setting motor speed 7x\n");
        run_motor_speed(7);

        MAP_SysCtlDelay(g_ui32SysClock / 3 );
        MAP_SysCtlDelay(g_ui32SysClock / 3 );

        UARTprintf("setting motor speed 8x\n");
        run_motor_speed(8);

        MAP_SysCtlDelay(g_ui32SysClock / 3 );
        MAP_SysCtlDelay(g_ui32SysClock / 3 );

        UARTprintf("Braking motor!\n");
        run_motor_speed(0);
        MAP_SysCtlDelay(g_ui32SysClock / 3 );
        MAP_SysCtlDelay(g_ui32SysClock / 3 );
        /*
        if((MAP_PWMPulseWidthGet(PWM0_BASE, PWM_OUT_2) + g_ui32PWMIncrement) <=
               ((MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) * 8) / 10))
        {
            MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                                 MAP_PWMPulseWidthGet(PWM0_BASE, PWM_GEN_1) +
                                 g_ui32PWMIncrement);
        }
        else
        {
            MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, g_ui32PWMIncrement*5);
        }
        */


    }
}

//level from 1 to 10
void run_motor_speed(int level)
{
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, g_ui32PWMIncrement * level);
}
