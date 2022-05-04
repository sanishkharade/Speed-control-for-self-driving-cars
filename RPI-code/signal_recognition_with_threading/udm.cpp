/*
 * @file_name       :   udm.cpp
 * 
 * @brief           :   RTES Final Project Code
 * 
 * @author          :   Sanish Kharade
 *                      Tanmay Kothale 
 *                      Vishal Raj
 * 
 * @date            :   May 03, 2022
 * 
 * @references	    : 	https://github.com/undqurek/RaspberryPI_UltrasonicRangeFinder	
 */
 
/**********************************************************************/
/*                          LIBRARY FILES                             */
/**********************************************************************/
#include <pigpio.h>
#include <iostream>
#include <time.h>
#include "udm.h"
#include <signal.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

/**********************************************************************/
/*                  PRIVATE MACROS AND DEFINES                        */
/**********************************************************************/
#define TRIG 	23
#define ECHO 	24
#define speed 	17150

/**********************************************************************/
/*               NAMESPACE FOR IO OPERATIONS IN CPP                   */
/**********************************************************************/
using namespace std;

//to get current time
struct timeval tv;

double get_instant()
{
	gettimeofday(&tv, NULL);

    return (double)tv.tv_sec + (double)tv.tv_usec * 0.000001;
}

/*see documentation in udm.h*/
void init_udm()
{
    gpioSetMode(TRIG, PI_OUTPUT);
    gpioSetMode(ECHO, PI_INPUT);
}


/************************************************************************************
 * @brief   :   creates a delay
 *              
 * @param   :   value	-	time for which delay is requested
 * 		limit	-	max limit is 1 msec
 *
 * @return  :   true on success, false on failure
 *              
*************************************************************************************/
bool delay(int value, int limit = 1000000)
{
    for(int i = 0; gpioRead(ECHO) == value; ++i)
    {
        if(i >= limit)
            return false;
    }

    return true;
}

/*see documentation in udm.h*/
double get_distance()
{
    gpioWrite(TRIG, 0);
    usleep(100000);
    gpioWrite(TRIG, 1);
    usleep(10);
    gpioWrite(TRIG, 0);

    if(delay(0))
    {
        double start_pulse = get_instant();

        if(delay(1))
        {
            double end_pulse = get_instant();

            double time = end_pulse - start_pulse;
            double distance = time * speed;

            return distance;
        }
    }

    return 0.0 / 0.0;
}
