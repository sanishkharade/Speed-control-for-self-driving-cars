/***********************************************************************************************************************
* File Name    : main.c
* Project      : RTES Final Project FreeRTOS service implementation.
* Version      : 1.0.
* Description  : Contains the source code for creation of multiple tasks and usage of various synchronization
*                mechanisms between them for RTES project services.
* Author       : Vishal Raj & Sanish Kharade, referred from  https://github.com/akobyl/TM4C129_FreeRTOS_Demo,
*                TivaWare_C_Series-2.2.0.295 SDK examples.
* Creation Date: 4.30.22
***********************************************************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "drivers/pinout.h"
#include "utils/uartstdio.h"


// TivaWare includes
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"


// For UART
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"


// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "driverlib/pin_map.h"

// For PWM
#include "driverlib/pwm.h"

// For Timer
//#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

#define SERIAL_TASK_PRIORITY    (1)
#define LED_TASK_PRIORITY       (1)
#define MOTOR_TASK_PRIORITY     (1)


#define SERIAL_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE)
#define LED_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE)
#define MOTOR_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE)
#define SCALING_FACTOR             100000//1000 //for 1ms timer
#define TIMER_FREQ                 2500
#define CLOCK_FREQ                 120000000
#define TIMER_LOAD_VALUE           (CLOCK_FREQ/SCALING_FACTOR)

#define SPEED_FOR_RED       0
#define SPEED_FOR_YELLOW    4
#define SPEED_FOR_GREEN     6

uint32_t ui32Period,pom = 0;


//Task and function prototypes
void LEDTask(void *pvParameters);
void SerialTask(void *pvParameters);
void MOTORTask(void *pvParameters);
void pwm_control(void);
void run_motor_speed(int level);
static void calculate_wcet(uint32_t start, uint32_t end,uint32_t *WCET,uint8_t id);


uint32_t g_ui32SysClock;
xQueueHandle g_pMyQueue;
xQueueHandle g_pMotorQueue;
// For PWM
uint32_t g_ui32PWMIncrement;

typedef enum
{
    NONE = 0,
    RED,
    YELLOW,
    GREEN
}color_t;


color_t color = NONE;
uint8_t current_color = 0;
char cMessage;
uint8_t speed = 5;

/***********************************************************************************************
* Name          : UART3IntHandler
* Description   : UART interrupt handler.
* Parameters    : None
* RETURN        : None
***********************************************************************************************/
void UART3IntHandler(void)
{
    uint32_t ui32Status;

    // Get the interrrupt status.
    ui32Status = MAP_UARTIntStatus(UART3_BASE, true);

    // Clear the asserted interrupts.
    MAP_UARTIntClear(UART3_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
    char c = 0;
    while(MAP_UARTCharsAvail(UART3_BASE))
    {
        // Read the next character from the UART and write it back to the UART.
        c =  MAP_UARTCharGetNonBlocking(UART3_BASE);
        current_color = c;

        if(xQueueSend(g_pMotorQueue, &current_color, portMAX_DELAY) != pdPASS)
        {
            UARTprintf("\nQueue full. This should never happen.\n");
            while(1)
            {
            }
        }
    }
}

/***********************************************************************************************
* Name          : UARTSend
* Description   : function for sending data through UART to raspberry pi.
* Parameters    : @param pui8Buffer Buffer reference
*                 @param ui32Count size of buffer to be send
* RETURN        : None
***********************************************************************************************/
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    // Loop while there are more characters to send.
    while(ui32Count--)
    {
        // Write the next character to the UART.
        MAP_UARTCharPutNonBlocking(UART3_BASE, *pui8Buffer++);
    }
}

/***********************************************************************************************
* Name          : UART_init
* Description   : function for initializing the UART module for inter board communication.
* Parameters    : None
* RETURN        : None
***********************************************************************************************/
void UART_init(void)
{
    // Enable the peripherals used by this example.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable processor interrupts.
    MAP_IntMasterEnable();

    // Set GPIO A0 and A1 as UART pins.
    MAP_GPIOPinConfigure(GPIO_PA4_U3RX);
    MAP_GPIOPinConfigure(GPIO_PA5_U3TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Configure the UART for 115,200, 8-N-1 operation.
    MAP_UARTConfigSetExpClk(UART3_BASE, g_ui32SysClock, 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    // Enable the UART interrupt.
    MAP_IntEnable(INT_UART3);
    MAP_UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT);
}

/***********************************************************************************************
* Name          : ConfigureUART
* Description   : function for initializing the UART module for logging and profiling services
* Parameters    : None
* RETURN        : None
***********************************************************************************************/
void ConfigureUART(void)
{
    // Enable the GPIO Peripheral used by the UART.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

/***********************************************************************************************
* Name          : PWM_init
* Description   : function for initializing the PWM module.
* Parameters    : None
* RETURN        : None
***********************************************************************************************/
void PWM_init(){

    uint32_t ui32PWMClockRate;

    // The PWM peripheral must be enabled for use.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    // Enable the GPIO port that is used for the PWM output.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Configure the GPIO pad for PWM function on pins PF2 and PF3.
    MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);
    MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    // Set the PWM clock to be SysClk / 8.
    MAP_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);

    //PWM generator period.
    //120Mhz ,15Mhz PWM
    ui32PWMClockRate = g_ui32SysClock / 8;

    //6Khz increment
    g_ui32PWMIncrement = ((ui32PWMClockRate / 250) / 10); //10% increment

    // Configure PWM2 to count up/down without synchronization.
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                        PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the PWM period to 250Hz.
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, (ui32PWMClockRate / 250));


    // Set PWM2 to a duty cycle of 60%.
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                         g_ui32PWMIncrement * 6); //start at 60%


    // Enable the PWM Out Bit 2 (PF2)
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);


    // Enable the PWM generator block.
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

/***********************************************************************************************
* Name          : vHWTimerInit
* Description   : Function for initializing TimerA0 module.
* Parameters    : None
* RETURN        : None
***********************************************************************************************/
void vHWTimerInit(void){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    ui32Period = (g_ui32SysClock / SCALING_FACTOR);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period);

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();

    TimerEnable(TIMER0_BASE, TIMER_A);

}

/***********************************************************************************************
* Name          : calculate_wcet
* Description   : Function for computing the WCET for each service base on start time, end time
*                 and function id.
* Parameters    : @param start - start time of service.
*                 @param end - end time of service execution.
*                 @param WCET - current WCET for the service.
*                 @param id - service id.
* RETURN        : None
***********************************************************************************************/
void calculate_wcet(uint32_t start, uint32_t end,uint32_t *WCET,uint8_t id){

    uint32_t Exec_t = 0;

    Exec_t = (end - start + TIMER_LOAD_VALUE) % TIMER_LOAD_VALUE;

    if(Exec_t > *WCET){

        *WCET = Exec_t;
        taskENTER_CRITICAL();

        //For x1 frequency.
        UARTprintf("==Service_%d WCET time:%d us==\n", id, (Exec_t*8)/1000);

        taskEXIT_CRITICAL();
    }

}

/***********************************************************************************************
* Name          : main
* Description   : entry point function of initialize all modules and create threads.
* Parameters    : None
* RETURN        : exit status of program.
***********************************************************************************************/
int main(void)
{
    // Initialize system clock to 120 MHz
    //uint32_t output_clock_rate_hz;
    g_ui32SysClock = ROM_SysCtlClockFreqSet(
                               (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                               SYSTEM_CLOCK);
    ASSERT(g_ui32SysClock == SYSTEM_CLOCK);

    UART_init();
    g_pMyQueue = xQueueCreate(5, sizeof(color_t));
    g_pMotorQueue = xQueueCreate(2, sizeof(uint32_t));

    //Enable clock
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    //Check if h/w access enabled
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
    {
    }

    //Set output
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_3);

    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_PIN_3);


    vHWTimerInit();

    PWM_init();

    // Initialize the GPIO pins for the Launchpad
    PinoutSet(false, false);

    UARTStdioConfig(0, 57600, SYSTEM_CLOCK);
    // Create tasks
    xTaskCreate(LEDTask, (const portCHAR *)"LEDs",
                LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, NULL);

    xTaskCreate(SerialTask, (const portCHAR *)"Serial",
               SERIAL_TASK_STACK_SIZE, NULL, SERIAL_TASK_PRIORITY, NULL);

    xTaskCreate(MOTORTask, (const portCHAR *)"Motor",
                MOTOR_TASK_STACK_SIZE, NULL, MOTOR_TASK_PRIORITY, NULL );

    vTaskStartScheduler();

    // Code should never reach this point
    return 0;
}

/***********************************************************************************************
* Name          : Timer0AIntHandler
* Description   : Interrupt handler for Timer0A.
* Parameters    : None
* RETURN        : None
***********************************************************************************************/
void Timer0AIntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    pom++;

}

/***********************************************************************************************
* Name          : MOTORTask
* Description   : The motor control service to vary motor speed according to UART input.
* Parameters    : @param pvParameters - thread parameter.
* RETURN        : None
***********************************************************************************************/
void MOTORTask(void *pvParameters){

    for(;;){

        pwm_control();

    }

}

/***********************************************************************************************
* Name          : pwm_control
* Description   : The function implementing motor speed control according to UART input.
* Parameters    : None
* RETURN        : None
***********************************************************************************************/
void pwm_control(void){

    uint8_t queueData = 0;
    TickType_t xStartTime,xEndTime;
    static uint32_t WCET = 0;

    if(xQueueReceive(g_pMotorQueue, &queueData, portMAX_DELAY) == pdPASS){

        /*Start time log*/
        xStartTime = TimerValueGet(TIMER0_BASE, TIMER_A);;

        if(queueData == RED)
            run_motor_speed(SPEED_FOR_RED);
        else if(queueData == YELLOW)
            run_motor_speed(SPEED_FOR_YELLOW);
        else if(queueData == GREEN)
            run_motor_speed(SPEED_FOR_GREEN);
        else
            run_motor_speed(SPEED_FOR_GREEN);

        /*End time log*/
        xEndTime = TimerValueGet(TIMER0_BASE, TIMER_A);;

        calculate_wcet(xStartTime,xEndTime, &WCET, 3);
    }

    //Send data to Rpi
    if(xQueueSend(g_pMyQueue, &queueData, portMAX_DELAY) != pdPASS)
    {
        // Error. The queue should never be full. If so print the
        // error message on UART and wait for ever.

        while(1)
        {
        }
    }

}

/***********************************************************************************************
* Name          : run_motor_speed
* Description   : Sets the PWM value for motor according to required level.
* Parameters    : @param level - motor speed from 0 to 10 i.e 0 to 100%.
* RETURN        : None
***********************************************************************************************/
void run_motor_speed(int level)
{
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, g_ui32PWMIncrement * level);
}


/***********************************************************************************************
* Name          : LEDTask
* Description   : The task to control green and red led status according to UART input.
* Parameters    : @param pvParameters - thread parameter.
* RETURN        : None
***********************************************************************************************/
void LEDTask(void *pvParameters)
{
    TickType_t xStartTime,xEndTime;
    uint32_t WCET = 0;

    for (;;)
    {
        /*Start time log*/
        xStartTime = TimerValueGet(TIMER0_BASE, TIMER_A);

        switch(color)
        {
            case NONE:
                LEDWrite(0x0F, 0x01);
                break;

            case RED:
                LEDWrite(0x0F, 0x02);

                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_PIN_3);

                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x00);

                break;

            case YELLOW:
                LEDWrite(0x0F, 0x04);
                break;

            case GREEN:
                LEDWrite(0x0F, 0x08);

                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);

                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, 0x00);
                break;

            default:
                LEDWrite(0x0F, 0x00);
                break;

        }

        /*End time log*/
        xEndTime = TimerValueGet(TIMER0_BASE, TIMER_A);


        calculate_wcet(xStartTime,xEndTime, &WCET, 1);

    }
}

/***********************************************************************************************
* Name          : SerialTask
* Description   : The task to send the acknowladgement back to Rpi after motor and LED actutation
*                 is complete.
* Parameters    : @param pvParameters - thread parameter.
* RETURN        : None
***********************************************************************************************/
void SerialTask(void *pvParameters)
{
    // Set up the UART which is connected to the virtual COM port

    TickType_t xStartTime,xEndTime;
    uint32_t WCET = 0;

    for (;;)
    {

        if(xQueueReceive(g_pMyQueue, &color, portMAX_DELAY) == pdPASS)
        {
            /*Start time log*/
            xStartTime = TimerValueGet(TIMER0_BASE, TIMER_A);;

            UARTSend((uint8_t *)(&color), 1);

            /*End time log*/
            xEndTime = TimerValueGet(TIMER0_BASE, TIMER_A);;

            calculate_wcet(xStartTime,xEndTime, &WCET, 2);

        }

    }
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
//[EOF]
