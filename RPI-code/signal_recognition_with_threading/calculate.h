/*
 * @file_name       :   calculate.h
 * 
 * @brief           :   RTES Final Project Code
 * 
 * @author          :   Sanish Kharade
 *                      Tanmay Kothale 
 *                      Vishal Raj
 * 
 * @date            :   May 03, 2022
 * 
 */
#ifndef _CALCULATE_H_
#define _CALCULATE_H_

/**********************************************************************/
/*                          LIBRARY FILES                             */
/**********************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "process.h"
#include "capture.h"

/**********************************************************************/
/*                  PRIVATE MACROS AND DEFINES                        */
/**********************************************************************/
#define CURRENT_SPEED_OF_CAR 	(9.834880)
#define TOTAL_TIME_REQD_TO_STOP (2.479339)
#define DISTANCE_REQD_TO_STOP	(24.38400)

/************************************************************************************
 * @brief   :   computes the time required for a moving vehicle to come to a complete
 * 				stop
 *              
 * @param   :   speed 		-	speed at which vehicle is traveling
 * 				distance	-	distance at which traffic light is detected
 *
 * @return  :   deadline	-	time required to complete all computations
 *              
*************************************************************************************/
float time_to_stop_in_sec(float speed, float distance);

/************************************************************************************
 * @brief   :   computes the time required to complete all tasks before coming to a 
 * 				full stop
 *              
 * @param   : 	distance			-	distance at which traffic light is detected
 *
 * @return  :   deadline_for_tasks	-	time required to complete all computations
 *              
*************************************************************************************/
float compute_deadline_to_complete_tasks (float distance);

#endif /*_CALCULATE_H_*/
