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
//        MAP_UARTCharPutNonBlocking(UART3_BASE, c );
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