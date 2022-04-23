/***********************************************************************************************************************
* File Name    : user_tasks.c
* Project      : RTES Exercise 5 FreeRTOS multitasking implementation.
* Version      : 1.0.
* Description  : Contains the source code for creation of multiple threads and usage of various synchronization
*                mechanisms between them.
* Author       : Vishal Raj and Nihal Thirunakarasu referred from rt_simple_thread_improved from
*                http://mercury.pr.erau.edu/~siewerts/cec450/code/,
*                RTES independent study report by Nishant bhat and https://github.com/akobyl/TM4C129_FreeRTOS_Demo.
* Creation Date: 4.1.22
***********************************************************************************************************************/
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "stack_macros.h"


#include <stdio.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "utils/uartstdio.h"
#include "user_task.h"

#include "semphr.h"

int idx = 0, jdx = 1;
int fib = 0, fib0 = 0, fib1 = 1;

#define FIB_LIMIT_FOR_32_BIT 47
int seqIterations = FIB_LIMIT_FOR_32_BIT;

#define FIB_TEST(seqCnt, iterCnt) \
    for(idx=0; idx < iterCnt; idx++) \
    { \
        while(jdx < seqCnt) \
        { \
            if (jdx == 0) \
            { \
                fib = 1; \
            } \
            else \
            { \
                fib0 = fib1; \
                fib1 = fib; \
                fib = fib0 + fib1; \
            } \
            jdx++; \
        } \
    }

#define TASK_1_TIMEOUT_30MS     30
#define TASK_2_TIMEOUT_80MS     80

#define FIB_1_ITR               2450
#define FIB_10_ITR              24500
#define FIB_40_ITR              77000

//#define FIB_Q2_x1_ITR           24500
//#define FIB_Q2_x2_ITR           245
//#define FIB_Q3_x100_ITR         24

#define FIB_Q2_x1_ITR           22000 // 10ms
#define FIB_Q2_x2_ITR           220   // 0.1ms
#define FIB_Q3_x100_ITR         22    // 0.01ms


static void vService_1(void* pvParameters);
static void vService_2(void* pvParameters);
static void vService_3(void* pvParameters);
static void vService_4(void* pvParameters);
static void vService_5(void* pvParameters);
static void vService_6(void* pvParameters);
static void vService_7(void* pvParameters);
static void calculate_wcet(uint32_t start, uint32_t end,uint32_t *WCET,uint8_t id);
//static void delay(int n);

uint32_t WCET_arr[NUMBER_SERVICES];
uint32_t ServiceNumber_arr[NUMBER_SERVICES] = {0, 0, 0, 0, 0, 0, 0};

//global variables
QueueHandle_t xQueue1;
int var1 = 0;
int start_time = 0;

SemaphoreHandle_t xsemS1,xsemS2;

/***********************************************************************************************
* Name          : calculate_wcet
* Description   : Used to calculate the WCET of the task. It compares the previous value and if
*                 greater then it will update the new value else it will retain the older WCET
*                 value
* Parameters    : None
* RETURN        : None
***********************************************************************************************/
void calculate_wcet(uint32_t start, uint32_t end,uint32_t *WCET,uint8_t id){

    uint32_t Exec_t = 0;

    ServiceNumber_arr[id-1]++;

    Exec_t = end - start;

    if(Exec_t > *WCET){
        *WCET = Exec_t;
        taskENTER_CRITICAL();
        WCET_arr[id-1] = Exec_t;


            #if (Q2_x1 == 1)
           //For x1 frequency.
           UARTprintf("==Service_%d WCET time:%d ms==\n", id, (*WCET));
           #endif
           #if (Q2_x2 == 1)
           //For x1 frequency.
           UARTprintf("==Service_%d WCET time:%d us==\n", id, (*WCET)*10);
           #endif
           #if (Q3_x100 == 1)
           //For x1 frequency.
           UARTprintf("==Service_%d WCET time:%d us==\n", id, (*WCET)*10);
           #endif
        taskEXIT_CRITICAL();
    }

}

/***********************************************************************************************
* Name          : vStartUserTasks
* Description   : used to start user tasks and registers timer and its callback
* Parameters    : None
* RETURN        : None
***********************************************************************************************/
void vStartUserTasks(void)
{
    int i;

    TimerHandle_t xTimerCheck = NULL;
    TaskHandle_t xServiceHandle = NULL;
    BaseType_t xReturn;

    /* UART configuration */
    //UARTStdioConfig(0, 57600, SYSTEM_CLOCK);

    /* Led configuration */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
    {
    }

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

    //Timer created properly
    if(xTimerCheck != NULL){
        xTimerStart( xTimerCheck, 0);
    }

    /* Semaphore creation*/
    for(i=0;i<=NUMBER_SERVICES;i++){

        xsemS[i] = xSemaphoreCreateBinary();
        //check for error
        if(xsemS[i] == NULL){
            UARTprintf("Bin semaphore creation error!\n");
            for(;;);
           }
    }

    /* Create timer user task1 here*/
    xReturn = xTaskCreate( vService_1,
                               (const char *)"user_timer_task1",
                               (configMINIMAL_STACK_SIZE * 2),
                               ( void * ) 1,
                               ( tskIDLE_PRIORITY + 4 ),
                               &xServiceHandle );

    if(xReturn != pdPASS){//User timer task creation failed
        vTaskDelete(xServiceHandle);
    }

    /* Create timer user task2 here*/
    xReturn = xTaskCreate( vService_2,
                               (const char *)"user_timer_task2",
                               (configMINIMAL_STACK_SIZE * 2),
                               ( void * ) 1,
                               ( tskIDLE_PRIORITY + 3 ),
                               &xServiceHandle );

    if(xReturn != pdPASS){//User timer task creation failed
        vTaskDelete(xServiceHandle);
    }

    /* Create timer user task3 here*/
    xReturn = xTaskCreate( vService_3,
                               (const char *)"user_timer_task3",
                               (configMINIMAL_STACK_SIZE * 2),
                               ( void * ) 1,
                               ( tskIDLE_PRIORITY + 2 ),
                               &xServiceHandle );

    if(xReturn != pdPASS){//User timer task creation failed
        vTaskDelete(xServiceHandle);
    }

    /* Create timer user task4 here*/
    xReturn = xTaskCreate( vService_4,
                               (const char *)"user_timer_task4",
                               (configMINIMAL_STACK_SIZE * 2),
                               ( void * ) 1,
                               ( tskIDLE_PRIORITY + 3 ),
                               &xServiceHandle );

    if(xReturn != pdPASS){//User timer task creation failed
        vTaskDelete(xServiceHandle);
    }

    /* Create timer user task5 here*/
    xReturn = xTaskCreate( vService_5,
                               (const char *)"user_timer_task5",
                               (configMINIMAL_STACK_SIZE * 2),
                               ( void * ) 1,
                               ( tskIDLE_PRIORITY + 2 ),
                               &xServiceHandle );

    if(xReturn != pdPASS){//User timer task creation failed
        vTaskDelete(xServiceHandle);
    }

    /* Create timer user task6 here*/
    xReturn = xTaskCreate( vService_6,
                               (const char *)"user_timer_task6",
                               (configMINIMAL_STACK_SIZE * 2),
                               ( void * ) 1,
                               ( tskIDLE_PRIORITY + 3 ),
                               &xServiceHandle );

    if(xReturn != pdPASS){//User timer task creation failed
        vTaskDelete(xServiceHandle);
    }

    /* Create timer user task7 here*/
    xReturn = xTaskCreate( vService_7,
                               (const char *)"user_timer_task7",
                               (configMINIMAL_STACK_SIZE * 2),
                               ( void * ) 1,
                               ( tskIDLE_PRIORITY + 1 ),
                               &xServiceHandle );

    if(xReturn != pdPASS){//User timer task creation failed
        vTaskDelete(xServiceHandle);
    }

}

/***********************************************************************************************
* Name          : vService_1
* Description   : The task1 processing load which executes for 10ms for Q2_x1 and Q2_x2 with a
*                 frequency of 3Hz and 6Hz respectively. The task1 processing load which executes
*                 for 1ms for Q3_x100 with a frequency of 300Hz.
* Parameters    : Thread parameters
* RETURN        : None
***********************************************************************************************/
static void vService_1(void* pvParameters){

    TickType_t xStartTime,xEndTime;

    taskENTER_CRITICAL();
    UARTprintf("---Starting Service_1---\n");
    taskEXIT_CRITICAL();

    uint32_t WCET = 0;

    unsigned long long S1Cnt=0;

    while(1)
    {
        if( xSemaphoreTake(xsemS[1], portMAX_DELAY) == pdPASS){
            //UARTprintf("Task 1\n");
        }

        /*Log task start time:*/
        //UARTprintf(">>Service_1 start:%dms<<\n", xTaskGetTickCount());

        xStartTime = pom;

        S1Cnt++;

        #if (Q2_x1 == 1)
        FIB_TEST(seqIterations, FIB_Q2_x1_ITR);
        #endif
        #if (Q2_x2 == 1)
        FIB_TEST(seqIterations, FIB_Q2_x2_ITR);
        #endif
        #if (Q3_x100 == 1)
        FIB_TEST(seqIterations, FIB_Q3_x100_ITR);
        #endif

        xEndTime = pom;

        calculate_wcet(xStartTime,xEndTime, &WCET, 1);

        /*Log task end time:*/
        //UARTprintf("<<Service_1 end:%dms>>\n", xTaskGetTickCount());

        //UARTprintf("==Service_1 WCET time:%dms==\n", (*WCET));
    }
}

/***********************************************************************************************
* Name          : vService_2
* Description   : The task2 processing load which executes for 10ms for Q2_x1 and Q2_x2 with a
*                 frequency of 1Hz and 2Hz respectively. The task1 processing load which executes
*                 for 1ms for Q3_x100 with a frequency of 100Hz.
* Parameters    : Thread parameters
* RETURN        : None
***********************************************************************************************/
static void vService_2(void* pvParameters){

    TickType_t xStartTime,xEndTime;

    taskENTER_CRITICAL();
    UARTprintf("---Starting Service_2---\n");
    taskEXIT_CRITICAL();

    uint32_t WCET = 0;

    unsigned long long S2Cnt=0;

    while(1){

        if( xSemaphoreTake(xsemS[2], portMAX_DELAY) == pdPASS){
            //UARTprintf("Task 2\n");
        }

        /*Log task start time:*/
        //UARTprintf(">>--Service_2 start:%dms--<<\n", xTaskGetTickCount());

        xStartTime = pom;

        S2Cnt++;

        #if (Q2_x1 == 1)
        FIB_TEST(seqIterations, FIB_Q2_x1_ITR);
        #endif
        #if (Q2_x2 == 1)
        FIB_TEST(seqIterations, FIB_Q2_x2_ITR);
        #endif
        #if (Q3_x100 == 1)
        FIB_TEST(seqIterations, FIB_Q3_x100_ITR);
        #endif

        xEndTime = pom;

        calculate_wcet(xStartTime,xEndTime, &WCET, 2);

        /*Log task end time:*/
        //UARTprintf("<<--Service_2 end:%dms-->>\n", xTaskGetTickCount());

        //UARTprintf("==Service_2 WCET time:%dms==\n", (xEndTime - xStartTime));
    }
}

/***********************************************************************************************
* Name          : vService_3
* Description   : The task2 processing load which executes for 10ms for Q2_x1 and Q2_x2 with a
*                 frequency of 0.5Hz and 1Hz respectively. The task1 processing load which executes
*                 for 1ms for Q3_x100 with a frequency of 50Hz.
* Parameters    : Thread parameters
* RETURN        : None
***********************************************************************************************/
static void vService_3(void* pvParameters){

    TickType_t xStartTime,xEndTime;

    taskENTER_CRITICAL();
    UARTprintf("---Starting Service_3---\n");
    taskEXIT_CRITICAL();

    uint32_t WCET = 0;

    unsigned long long S3Cnt=0;

    while(1)
    {
        if( xSemaphoreTake(xsemS[3], portMAX_DELAY) == pdPASS){
            //UARTprintf("Task 1\n");
        }

        /*Log task start time:*/
        //UARTprintf(">>Service_3 start:%dms<<\n", xTaskGetTickCount());

        xStartTime = pom;

        S3Cnt++;

        #if (Q2_x1 == 1)
        FIB_TEST(seqIterations, FIB_Q2_x1_ITR);
        #endif
        #if (Q2_x2 == 1)
        FIB_TEST(seqIterations, FIB_Q2_x2_ITR);
        #endif
        #if (Q3_x100 == 1)
        FIB_TEST(seqIterations, FIB_Q3_x100_ITR);
        #endif

        xEndTime = pom;

        calculate_wcet(xStartTime,xEndTime, &WCET, 3);

        /*Log task end time:*/
        //UARTprintf("<<Service_3 end:%dms>>\n", xTaskGetTickCount());

        //UARTprintf("==Service_3 WCET time:%dms==\n", (xEndTime - xStartTime));
    }
}

/***********************************************************************************************
* Name          : vService_4
* Description   : The task4 processing load which executes for 10ms for Q2_x1 and Q2_x2 with a
*                 frequency of 1Hz and 2Hz respectively. The task1 processing load which executes
*                 for 1ms for Q3_x100 with a frequency of 100Hz.
* Parameters    : Thread parameters
* RETURN        : None
***********************************************************************************************/
static void vService_4(void* pvParameters){

    TickType_t xStartTime,xEndTime;

    taskENTER_CRITICAL();
    UARTprintf("---Starting Service_4---\n");
    taskEXIT_CRITICAL();

    uint32_t WCET = 0;

    unsigned long long S4Cnt=0;

    while(1){

        if( xSemaphoreTake(xsemS[4], portMAX_DELAY) == pdPASS){
            //UARTprintf("Task 2\n");
        }

        /*Log task start time:*/
        //UARTprintf(">>--Service_4 start:%dms--<<\n", xTaskGetTickCount());

        xStartTime = pom;

        S4Cnt++;

        #if (Q2_x1 == 1)
        FIB_TEST(seqIterations, FIB_Q2_x1_ITR);
        #endif
        #if (Q2_x2 == 1)
        FIB_TEST(seqIterations, FIB_Q2_x2_ITR);
        #endif
        #if (Q3_x100 == 1)
        FIB_TEST(seqIterations, FIB_Q3_x100_ITR);
        #endif

        xEndTime = pom;

        calculate_wcet(xStartTime,xEndTime, &WCET, 4);

        /*Log task end time:*/
        //UARTprintf("<<--Service_4 end:%dms-->>\n", xTaskGetTickCount());

        //UARTprintf("==Service_4 WCET time:%dms==\n", (xEndTime - xStartTime));
    }
}

/***********************************************************************************************
* Name          : vService_5
* Description   : The task5 processing load which executes for 10ms for Q2_x1 and Q2_x2 with a
*                 frequency of 0.5Hz and 1Hz respectively. The task1 processing load which executes
*                 for 1ms for Q3_x100 with a frequency of 50Hz.
* Parameters    : Thread parameters
* RETURN        : None
***********************************************************************************************/
static void vService_5(void* pvParameters){

    TickType_t xStartTime,xEndTime;

    taskENTER_CRITICAL();
    UARTprintf("---Starting Service_5---\n");
    taskEXIT_CRITICAL();

    uint32_t WCET = 0;

    unsigned long long S5Cnt=0;

    while(1)
    {
        if( xSemaphoreTake(xsemS[5], portMAX_DELAY) == pdPASS){
            //UARTprintf("Task 1\n");
        }

        /*Log task start time:*/
        //UARTprintf(">>Service_5 start:%dms<<\n", xTaskGetTickCount());

        xStartTime = pom;

        S5Cnt++;

        #if (Q2_x1 == 1)
        FIB_TEST(seqIterations, FIB_Q2_x1_ITR);
        #endif
        #if (Q2_x2 == 1)
        FIB_TEST(seqIterations, FIB_Q2_x2_ITR);
        #endif
        #if (Q3_x100 == 1)
        FIB_TEST(seqIterations, FIB_Q3_x100_ITR);
        #endif

        xEndTime = pom;

        calculate_wcet(xStartTime,xEndTime, &WCET, 5);

        /*Log task end time:*/
        //UARTprintf("<<Service_5 end:%dms>>\n", xTaskGetTickCount());

        //UARTprintf("==Service_5 WCET time:%dms==\n", (xEndTime - xStartTime));
    }
}

/***********************************************************************************************
* Name          : vService_6
* Description   : The task6 processing load which executes for 10ms for Q2_x1 and Q2_x2 with a
*                 frequency of 1Hz and 2Hz respectively. The task1 processing load which executes
*                 for 1ms for Q3_x100 with a frequency of 100Hz.
* Parameters    : Thread parameters
* RETURN        : None
***********************************************************************************************/
static void vService_6(void* pvParameters){

    TickType_t xStartTime,xEndTime;

    taskENTER_CRITICAL();
    UARTprintf("---Starting Service_6---\n");
    taskEXIT_CRITICAL();

    uint32_t WCET = 0;

    unsigned long long S6Cnt=0;

    while(1){

        if( xSemaphoreTake(xsemS[6], portMAX_DELAY) == pdPASS){
            //UARTprintf("Task 2\n");
        }

        /*Log task start time:*/
        //UARTprintf(">>--Service_6 start:%dms--<<\n", xTaskGetTickCount());

        xStartTime = pom;

        S6Cnt++;

        #if (Q2_x1 == 1)
        FIB_TEST(seqIterations, FIB_Q2_x1_ITR);
        #endif
        #if (Q2_x2 == 1)
        FIB_TEST(seqIterations, FIB_Q2_x2_ITR);
        #endif
        #if (Q3_x100 == 1)
        FIB_TEST(seqIterations, FIB_Q3_x100_ITR);
        #endif

        xEndTime = pom;

        calculate_wcet(xStartTime,xEndTime, &WCET, 6);

        /*Log task end time:*/
        //UARTprintf("<<--Service_6 end:%dms-->>\n", xTaskGetTickCount());

        //UARTprintf("==Service_6 WCET time:%dms==\n", (xEndTime - xStartTime));
    }
}

/***********************************************************************************************
* Name          : vService_7
* Description   : The task7 processing load which executes for 10ms for Q2_x1 and Q2_x2 with a
*                 frequency of 0.1Hz and 0.2Hz respectively. The task1 processing load which
*                 executes for 1ms for Q3_x100 with a frequency of 10Hz.
* Parameters    : Thread parameters
* RETURN        : None
***********************************************************************************************/
static void vService_7(void* pvParameters){

    TickType_t xStartTime,xEndTime;

    taskENTER_CRITICAL();
    UARTprintf("---Starting Service_7---\n");
    taskEXIT_CRITICAL();

    uint32_t WCET = 0;

    unsigned long long S7Cnt=0;

    while(1){

        if( xSemaphoreTake(xsemS[7], portMAX_DELAY) == pdPASS){
            //UARTprintf("Task 2\n");
        }

        /*Log task start time:*/
        //UARTprintf(">>--Service_7 start:%dms--<<\n", xTaskGetTickCount());

        xStartTime = pom;

        S7Cnt++;

        #if (Q2_x1 == 1)
        FIB_TEST(seqIterations, FIB_Q2_x1_ITR);
        #endif
        #if (Q2_x2 == 1)
        FIB_TEST(seqIterations, FIB_Q2_x2_ITR);
        #endif
        #if (Q3_x100 == 1)
        FIB_TEST(seqIterations, FIB_Q3_x100_ITR);
        #endif

        xEndTime = pom;

        calculate_wcet(xStartTime,xEndTime, &WCET, 7);

        /*Log task end time:*/
        //UARTprintf("<<--Service_7 end:%dms-->>\n", xTaskGetTickCount());

        //UARTprintf("==Service_7 WCET time:%dms==\n", (xEndTime - xStartTime));
    }
}

//[EOF]
