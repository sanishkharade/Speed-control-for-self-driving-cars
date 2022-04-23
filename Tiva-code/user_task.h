/***********************************************************************************************************************
* File Name    : user_tasks.h
* Project      : RTES Exercise 5 FreeRTOS multitasking implementation.
* Version      : 1.0.
* Description  : Header for user_task source file
* Author       : Vishal Raj and Nihal Thirunakarasu
* Creation Date: 4.1.22
***********************************************************************************************************************/

#ifndef USER_TASK_H_
#define USER_TASK_H_
#include "semphr.h"

#define Q2_x1      1
#define Q2_x2      0
#define Q3_x100    0


// Keep this 1000 for interrupt to hit every ms
#if (Q2_x1 == 1)
//For x1 frequency.
#define SCALING_FACTOR      1000
#endif
#if (Q2_x2 == 1)
//For x1 frequency.
#define SCALING_FACTOR      100000
#endif
#if (Q3_x100 == 1)
//For x1 frequency.
#define SCALING_FACTOR      100000
#endif


#define PROGRAM_TIMEOUT 30 // The program will terminate after 30 sec

#define NUMBER_SERVICES         8

void vStartUserTasks(void);

extern uint32_t pom;
extern SemaphoreHandle_t xsemS[];
extern uint32_t WCET_arr[NUMBER_SERVICES];
extern uint32_t ServiceNumber_arr[NUMBER_SERVICES];


#endif /* USER_TASK_H_ */
