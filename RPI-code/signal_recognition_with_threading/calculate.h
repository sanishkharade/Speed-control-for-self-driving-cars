#ifndef _CALCULATE_H_
#define _CALCULATE_H_

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "process.h"
#include "capture.h"

#define CURRENT_SPEED_OF_CAR 	(9.834880)
#define TOTAL_TIME_REQD_TO_STOP (2.479339)
#define DISTANCE_REQD_TO_STOP	(24.38400)

float time_to_stop_in_sec(float speed, float distance);
float compute_deadline_to_complete_tasks (float distance);

#endif /*_CALCULATE_H_*/
