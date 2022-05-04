/*
 * @file_name       :   calculate.cpp
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
 
/*LIBRARY FILES*/
#include "calculate.h"

/*see documentation in calculate.h*/
float time_to_stop_in_sec(float speed, float distance)
{
	float deadline; 
	
	deadline = distance/speed;
	
	return deadline;
}

/*see documentation in calculate.h*/
float compute_deadline_to_complete_tasks (float distance)
{
	float deadline, time_to_stop, deadline_for_tasks;
	
	time_to_stop = time_to_stop_in_sec(CURRENT_SPEED_OF_CAR, DISTANCE_REQD_TO_STOP);
	
	deadline = time_to_stop_in_sec(CURRENT_SPEED_OF_CAR, distance);
	
	deadline_for_tasks = deadline - time_to_stop;
	
	return deadline_for_tasks;
}
