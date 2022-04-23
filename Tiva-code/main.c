/***********************************************************************************************************************
* File Name    : user_tasks.c
* Project      : RTES Exercise 5 FreeRTOS multitasking implementation.
* Version      : 1.0.
* Description  : Contains the source code for creation of multiple threads and usage of various synchronization
*                mechanisms between them.
* Author       : Vishal Raj & Nihal Thirunakarasu referred from  https://github.com/akobyl/TM4C129_FreeRTOS_Demo.
* Creation Date: 4.1.22
***********************************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "main.h"
#include "drivers/pinout.h"
#include "utils/uartstdio.h"

// TivaWare includes
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "user_task.h"

//Timer headers
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

uint32_t output_clock_rate_hz;
uint32_t ui32Period, pom = 0;
SemaphoreHandle_t xsemS[NUMBER_SERVICES];


#define SERVICE_1_FREQ       30
#define SERVICE_2_FREQ       10
#define SERVICE_3_FREQ       5
#define SERVICE_4_FREQ       10
#define SERVICE_5_FREQ       5
#define SERVICE_6_FREQ       10
#define SERVICE_7_FREQ       1

#if (Q2_x1 == 1)
//For x1 frequency.
#define FREQ_MULTIPLIER      1
#endif
#if (Q2_x2 == 1)
//For x1 frequency.
#define FREQ_MULTIPLIER      10
#endif
#if (Q3_x100 == 1)
//For x1 frequency.
#define FREQ_MULTIPLIER      100
#endif


#define SERVICE_1_PERIOD     ((SCALING_FACTOR*10)/(SERVICE_1_FREQ))*FREQ_MULTIPLIER
#define SERVICE_2_PERIOD     ((SCALING_FACTOR*10)/(SERVICE_2_FREQ))*FREQ_MULTIPLIER
#define SERVICE_3_PERIOD     ((SCALING_FACTOR*10)/(SERVICE_3_FREQ))*FREQ_MULTIPLIER
#define SERVICE_4_PERIOD     ((SCALING_FACTOR*10)/(SERVICE_4_FREQ))*FREQ_MULTIPLIER
#define SERVICE_5_PERIOD     ((SCALING_FACTOR*10)/(SERVICE_5_FREQ))*FREQ_MULTIPLIER
#define SERVICE_6_PERIOD     ((SCALING_FACTOR*10)/(SERVICE_6_FREQ))*FREQ_MULTIPLIER
#define SERVICE_7_PERIOD     ((SCALING_FACTOR*10)/(SERVICE_7_FREQ))*FREQ_MULTIPLIER

int service_Freq_arr[NUMBER_SERVICES-1] = {30, 10, 5, 10, 5, 10, 1};

/***********************************************************************************************
* Name          : main
* Description   : used to create the user task creation function and start the scheduler along
*                 with clock configuration.
* Parameters    : None
* RETURN        : function return status
***********************************************************************************************/
int main(void)
{
    // Initialize system clock to 120 MHz
    uint32_t output_clock_rate_hz;
    output_clock_rate_hz = ROM_SysCtlClockFreqSet(
                               (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                               SYSTEM_CLOCK);
    ASSERT(output_clock_rate_hz == SYSTEM_CLOCK);

    /* UART configuration */
    UARTStdioConfig(0, 57600, SYSTEM_CLOCK);

    /* TimerA configuration*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    ui32Period = ((output_clock_rate_hz / SCALING_FACTOR)/*/1000->ms*/);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period);

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();

    TimerEnable(TIMER0_BASE, TIMER_A);
    /* TimerA configuration*/

    // Initialize the GPIO pins for the Launchpad
    PinoutSet(false, false);

    //timer_init();

    #if (Q2_x1 == 1)
    //For x1 frequency.
    UARTprintf("\n\nQuestion 2 (FreeRTOS)\nx1 Frequency Scaled (seqgen.c):");
    UARTprintf("\n\nTest Starting......\n-------------------\n");
    #endif
    #if (Q2_x2 == 1)
    //For x1 frequency.
    UARTprintf("\n\nQuestion 2 (FreeRTOS)\nx2 Frequency Scaled (seqgen2x.c):");
    UARTprintf("\n\nTest Starting......\n-------------------\n");
    #endif
    #if (Q3_x100 == 1)
    //For x1 frequency.
    UARTprintf("\n\nQuestion 3 (FreeRTOS)\nx100 Frequency Scaled:");
    UARTprintf("\n\nTest Starting......\n-------------------\n");
    #endif

    vStartUserTasks();

    vTaskStartScheduler();
    


    // Code should never reach this point
    return 0;
}


/***********************************************************************************************
* Name          : Timer0AIntHandler
* Description   : The timer callback function which executes after every timer expiry for Q3.
*                 This also dispatched the two tasks by releasing task specific semaphores.
* Parameters    : Timer handler of the timer to which the callback is registered.
* RETURN        : None
***********************************************************************************************/
void Timer0AIntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    pom++;

    int i;

    if (pom > (PROGRAM_TIMEOUT*SCALING_FACTOR))
    {
        vTaskSuspendAll();
        IntDisable(INT_TIMER0A);

        UARTprintf("\n\nTest Completed!!\n");

        UARTprintf("\nResults:\n--------\n");

        for (i=0; i<NUMBER_SERVICES-1; i++)
        {
            #if (Q2_x1 == 1)
            //For x1 frequency.
            UARTprintf("\nService Number %d: WCET = %d ms :: Number of times executed: %d :: Service Frequency: %d.%01d Hz", i+1, WCET_arr[i], ServiceNumber_arr[i], service_Freq_arr[i]/10, service_Freq_arr[i]%10);
            #endif
            #if (Q2_x2 == 1)
            //For x1 frequency.
            UARTprintf("\nService Number %d: WCET = %d us :: Number of times executed: %d :: Service Frequency: %d.%01d Hz", i+1, WCET_arr[i]*10, ServiceNumber_arr[i], service_Freq_arr[i]/10, service_Freq_arr[i]%10);
            #endif
            #if (Q3_x100 == 1)
            //For x1 frequency.
            UARTprintf("\nService Number %d: WCET = %d us :: Number of times executed: %d :: Service Frequency: %d.%01d Hz", i+1, WCET_arr[i]*10, ServiceNumber_arr[i], service_Freq_arr[i]/10, service_Freq_arr[i]%10);
            #endif

        }
        UARTprintf("\n\n**************\n");
    }

    if((pom % SERVICE_1_PERIOD) == 0)
        xSemaphoreGive(xsemS[1]);

    if((pom % SERVICE_2_PERIOD) == 0)
        xSemaphoreGive(xsemS[2]);

    if((pom % SERVICE_3_PERIOD) == 0)
        xSemaphoreGive(xsemS[3]);

    if((pom % SERVICE_4_PERIOD) == 0)
        xSemaphoreGive(xsemS[4]);

    if((pom % SERVICE_5_PERIOD) == 0)
        xSemaphoreGive(xsemS[5]);

    if((pom % SERVICE_6_PERIOD) == 0)
        xSemaphoreGive(xsemS[6]);

    if((pom % SERVICE_7_PERIOD) == 0)
        xSemaphoreGive(xsemS[7]);
}

/*  ASSERT() Error function
 *
 *  failed ASSERTS() from driverlib/debug.h are executed in this function
 */
void __error__(char *pcFilename, uint32_t ui32Line)
{
    // Place a breakpoint here to capture errors until logging routine is finished
    while (1)
    {
    }
}
