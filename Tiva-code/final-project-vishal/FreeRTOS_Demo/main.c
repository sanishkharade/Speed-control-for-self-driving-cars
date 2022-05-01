////**************************My code******************************
/* FreeRTOS 10 Tiva Demo
 *
 * main.c
 *
 * Andy Kobyljanec
 *
 * This is a simple demonstration project of FreeRTOS 8.2 on the Tiva Launchpad
 * EK-TM4C1294XL.  TivaWare driverlib sourcecode is included.
 */

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
uint32_t ui32Period,pom = 0;


// Demo Task declarations
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
//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UART3IntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = MAP_UARTIntStatus(UART3_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    MAP_UARTIntClear(UART3_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    char c = 0;
    while(MAP_UARTCharsAvail(UART3_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        c =  MAP_UARTCharGetNonBlocking(UART3_BASE);
        current_color = c;
        //printf("c = %d\n",c);
        //MAP_UARTCharPutNonBlocking(UART3_BASE, c);
        //MAP_UARTCharPutNonBlocking(UART3_BASE, MAP_UARTCharGetNonBlocking(UART3_BASE));
        //
        // Blink the LED to show a character transfer is occuring.
        //
//        if(xQueueSend(g_pMyQueue, &c, portMAX_DELAY) != pdPASS)
//        {
//            // Error. The queue should never be full. If so print the
//            // error message on UART and wait for ever.
//
//            UARTprintf("\nQueue full. This should never happen.\n");
//            while(1)
//            {
//            }
//        }

        if(xQueueSend(g_pMotorQueue, &current_color, portMAX_DELAY) != pdPASS)
        {
            UARTprintf("\nQueue full. This should never happen.\n");
            while(1)
            {
            }
        }
        //MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        //SysCtlDelay(g_ui32SysClock / (1000 * 3));

        //
        // Turn off the LED
        //
        //MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
    }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        MAP_UARTCharPutNonBlocking(UART3_BASE, *pui8Buffer++);
    }
}
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

void PWM_init(){

    uint32_t ui32PWMClockRate;

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
                         g_ui32PWMIncrement * 6); //start at 60%

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
}

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

void calculate_wcet(uint32_t start, uint32_t end,uint32_t *WCET,uint8_t id){

    uint32_t Exec_t = 0;

    Exec_t = (end - start + TIMER_LOAD_VALUE) % TIMER_LOAD_VALUE;

    //UARTprintf("==Service_%d WCET time:%d us==\n", id, (Exec_t*8)/1000);

    if(Exec_t > *WCET){

        *WCET = Exec_t;
        taskENTER_CRITICAL();

        //For x1 frequency.
        UARTprintf("==Service_%d WCET time:%d us==\n", id, (Exec_t*8)/1000);

        taskEXIT_CRITICAL();
    }

}

// Main function
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

    /*--Vishals part--*/

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

    //ConfigureUART();

    //UARTprintf("Starting program...\n");

    PWM_init();

    /*--Vishals part--*/

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


void Timer0AIntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    pom++;

}

void MOTORTask(void *pvParameters){

    for(;;){

        pwm_control();

    }

}

//Test comment
uint8_t speed = 5;

void pwm_control(void){

    uint8_t queueData = 0;
    TickType_t xStartTime,xEndTime;
    static uint32_t WCET = 0;

    if(xQueueReceive(g_pMotorQueue, &queueData, portMAX_DELAY) == pdPASS){

        /*Start time log*/
        xStartTime = TimerValueGet(TIMER0_BASE, TIMER_A);;

        if(queueData == 1)
            run_motor_speed(0);
        else if(queueData == 2)
            run_motor_speed(4);
        else if(queueData == 3)
            run_motor_speed(6);
        else
            run_motor_speed(6);

        /*End time log*/
        xEndTime = TimerValueGet(TIMER0_BASE, TIMER_A);;

        calculate_wcet(xStartTime,xEndTime, &WCET, 3);
    }

    //Send data to Rpi
    if(xQueueSend(g_pMyQueue, &queueData, portMAX_DELAY) != pdPASS)
    {
        // Error. The queue should never be full. If so print the
        // error message on UART and wait for ever.

        //UARTprintf("\nQueue full. This should never happen.\n");
        while(1)
        {
        }
    }

}

//level from 1 to 10
void run_motor_speed(int level)
{
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, g_ui32PWMIncrement * level);
}


// Flash the LEDs on the launchpa
void LEDTask(void *pvParameters)
{
    TickType_t xStartTime,xEndTime;
    uint32_t WCET = 0;

    for (;;)
    {
        /*Start time log*/
        xStartTime = TimerValueGet(TIMER0_BASE, TIMER_A);

        //printf("color = %d\n", color);
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

//        TickType_t execution_time = (xStartTime - xEndTime + TIMER_LOAD_VALUE) % TIMER_LOAD_VALUE;
//
//        UARTprintf("==Service_1 WCET time:%d ms==\n", ((execution_time*8)/1000));

        calculate_wcet(xStartTime,xEndTime, &WCET, 1);

    }
}


// Write text over the Stellaris debug interface UART port
void SerialTask(void *pvParameters)
{
    // Set up the UART which is connected to the virtual COM port
    //UARTStdioConfig(0, 57600, SYSTEM_CLOCK);
    //char cMessage;

    TickType_t xStartTime,xEndTime;
    uint32_t WCET = 0;

    for (;;)
    {
        //UARTprintf("Hello, world from FreeRTOS 10.2!\r\n");
        //vTaskDelay(5000 / portTICK_PERIOD_MS);

        if(xQueueReceive(g_pMyQueue, &color, portMAX_DELAY) == pdPASS)
        {
            //UARTprintf("Time = %d seconds\n\r", ulSeconds);

            /*Start time log*/
            xStartTime = TimerValueGet(TIMER0_BASE, TIMER_A);;

            UARTSend((uint8_t *)(&color), 1);

            /*End time log*/
            xEndTime = TimerValueGet(TIMER0_BASE, TIMER_A);;

            calculate_wcet(xStartTime,xEndTime, &WCET, 2);

            //UARTprintf("Hello, world from FreeRTOS 10.2!\r\n");
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

///**************************Sanish's code****************************/
///* FreeRTOS 10 Tiva Demo
// *
// * main.c
// *
// * Andy Kobyljanec
// *
// * This is a simple demonstration project of FreeRTOS 8.2 on the Tiva Launchpad
// * EK-TM4C1294XL.  TivaWare driverlib sourcecode is included.
// */
//#include <stdio.h>
//#include <stdint.h>
//#include <stdbool.h>
//#include "main.h"
//#include "drivers/pinout.h"
//#include "utils/uartstdio.h"
//
//
//// TivaWare includes
//#include "driverlib/sysctl.h"
//#include "driverlib/debug.h"
//#include "driverlib/rom.h"
//#include "driverlib/rom_map.h"
//
//
//// For UART
//#include "driverlib/uart.h"
//#include "driverlib/gpio.h"
//#include "inc/hw_ints.h"
//#include "inc/hw_memmap.h"
//
//
//// FreeRTOS includes
//#include "FreeRTOSConfig.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#include "driverlib/pin_map.h"
//
//#define SERIAL_TASK_PRIORITY    (1)
//#define LED_TASK_PRIORITY       (1)
//
//
//#define SERIAL_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE)
//#define LED_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE)
//
//
//// Demo Task declarations
//void LEDTask(void *pvParameters);
//void SerialTask(void *pvParameters);
//
//
//
//uint32_t g_ui32SysClock;
//xQueueHandle g_pMyQueue;
//
//typedef enum
//{
//    NONE = '0',
//    RED,
//    YELLOW,
//    GREEN
//}color_t;
//
//
//color_t color = NONE;
//char cMessage;
////*****************************************************************************
////
//// The UART interrupt handler.
////
////*****************************************************************************
//void
//UART3IntHandler(void)
//{
//    uint32_t ui32Status;
//
//    //
//    // Get the interrrupt status.
//    //
//    ui32Status = MAP_UARTIntStatus(UART3_BASE, true);
//
//    //
//    // Clear the asserted interrupts.
//    //
//    MAP_UARTIntClear(UART3_BASE, ui32Status);
//
//    //
//    // Loop while there are characters in the receive FIFO.
//    //
//    char c = 0;
//    while(MAP_UARTCharsAvail(UART3_BASE))
//    {
//        //
//        // Read the next character from the UART and write it back to the UART.
//        //
//        c =  MAP_UARTCharGetNonBlocking(UART3_BASE);
//        //printf("c = %d\n",c);
//        //MAP_UARTCharPutNonBlocking(UART3_BASE, c );
//        //MAP_UARTCharPutNonBlocking(UART3_BASE, MAP_UARTCharGetNonBlocking(UART3_BASE));
//        //
//        // Blink the LED to show a character transfer is occuring.
//        //
//        if(xQueueSend(g_pMyQueue, &c, portMAX_DELAY) != pdPASS)
//        {
//            // Error. The queue should never be full. If so print the
//            // error message on UART and wait for ever.
//
//            UARTprintf("\nQueue full. This should never happen.\n");
//            while(1)
//            {
//            }
//        }
//        //MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
//
//        //
//        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
//        //
//        //SysCtlDelay(g_ui32SysClock / (1000 * 3));
//
//        //
//        // Turn off the LED
//        //
//        //MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
//    }
//}
//
////*****************************************************************************
////
//// Send a string to the UART.
////
////*****************************************************************************
//void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
//{
//    //
//    // Loop while there are more characters to send.
//    //
//    while(ui32Count--)
//    {
//        //
//        // Write the next character to the UART.
//        //
//        MAP_UARTCharPutNonBlocking(UART3_BASE, *pui8Buffer++);
//    }
//}
//
//void UART_init(void)
//{
//    // Enable the peripherals used by this example.
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//
//    // Enable processor interrupts.
//    MAP_IntMasterEnable();
//
//    // Set GPIO A0 and A1 as UART pins.
//    MAP_GPIOPinConfigure(GPIO_PA4_U3RX);
//    MAP_GPIOPinConfigure(GPIO_PA5_U3TX);
//    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5);
//
//    // Configure the UART for 115,200, 8-N-1 operation.
//    MAP_UARTConfigSetExpClk(UART3_BASE, g_ui32SysClock, 9600,
//                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
//                             UART_CONFIG_PAR_NONE));
//
//    // Enable the UART interrupt.
//    MAP_IntEnable(INT_UART3);
//    MAP_UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT);
//}
//
//// Main function
//int main(void)
//{
//    // Initialize system clock to 120 MHz
//    //uint32_t output_clock_rate_hz;
//    g_ui32SysClock = ROM_SysCtlClockFreqSet(
//                               (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
//                                SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
//                               SYSTEM_CLOCK);
//    ASSERT(g_ui32SysClock == SYSTEM_CLOCK);
//
//    UART_init();
//    g_pMyQueue = xQueueCreate(5, sizeof(color_t));
//
//    // Initialize the GPIO pins for the Launchpad
//    PinoutSet(false, false);
//
//    // Create tasks
//    xTaskCreate(LEDTask, (const portCHAR *)"LEDs",
//                LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, NULL);
//
//    xTaskCreate(SerialTask, (const portCHAR *)"Serial",
//                SERIAL_TASK_STACK_SIZE, NULL, SERIAL_TASK_PRIORITY, NULL);
//
//    vTaskStartScheduler();
//
//    // Code should never reach this point
//    return 0;
//}
//
//
//// Flash the LEDs on the launchpa
//void LEDTask(void *pvParameters)
//{
//    for (;;)
//    {
//        //printf("color = %d\n", color);
//        switch(color)
//        {
//            case NONE:
//                LEDWrite(0x0F, 0x01);
//                break;
//
//            case RED:
//                LEDWrite(0x0F, 0x02);
//                break;
//
//            case YELLOW:
//                LEDWrite(0x0F, 0x04);
//                break;
//
//            case GREEN:
//                LEDWrite(0x0F, 0x08);
//                break;
//
//            default:
//                LEDWrite(0x0F, 0x00);
//                break;
//
//        }
////        // Turn on LED 1
////        LEDWrite(0x0F, 0x01);
//           //vTaskDelay(1000);
////
////        // Turn on LED 2
////        LEDWrite(0x0F, 0x02);
////        vTaskDelay(1000);
////
////        // Turn on LED 3
////        LEDWrite(0x0F, 0x04);
////        vTaskDelay(1000);
////
////        // Turn on LED 4
////        LEDWrite(0x0F, 0x08);
////        vTaskDelay(1000);
//    }
//}
//
//
//// Write text over the Stellaris debug interface UART port
//void SerialTask(void *pvParameters)
//{
//    // Set up the UART which is connected to the virtual COM port
//    UARTStdioConfig(0, 57600, SYSTEM_CLOCK);
//    //char cMessage;
//    for (;;)
//    {
//        //UARTprintf("Hello, world from FreeRTOS 10.2!\r\n");
//        //vTaskDelay(5000 / portTICK_PERIOD_MS);
//
//        if(xQueueReceive(g_pMyQueue, &color, portMAX_DELAY) == pdPASS)
//        {
//            //UARTprintf("Time = %d seconds\n\r", ulSeconds);
//            UARTSend((uint8_t *)(&color), 1);
//            //UARTprintf("Hello, world from FreeRTOS 10.2!\r\n");
//        }
//
//    }
//}
//
///*  ASSERT() Error function
// *
// *  failed ASSERTS() from driverlib/debug.h are executed in this function
// */
//void __error__(char *pcFilename, uint32_t ui32Line)
//{
//    // Place a breakpoint here to capture errors until logging routine is finished
//    while (1)
//    {
//    }
//}
