#include "calculate.h"

float time_to_stop_in_sec(float speed, float distance)
{
	float deadline; 
	
	deadline = distance/speed;
	
	return deadline;
}

float compute_deadline_to_complete_tasks (float distance)
{
	float deadline, time_to_stop, deadline_for_tasks;
	
	time_to_stop = time_to_stop_in_sec(CURRENT_SPEED_OF_CAR, DISTANCE_REQD_TO_STOP);
	
	deadline = time_to_stop_in_sec(CURRENT_SPEED_OF_CAR, distance);
	
	deadline_for_tasks = deadline - time_to_stop;
	
	return deadline_for_tasks;
}
