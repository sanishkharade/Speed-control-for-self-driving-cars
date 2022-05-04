/*
 * @file_name       :   capture.cpp
 * 
 * @brief           :   RTES Final Project Code
 * 
 * @author          :   Sanish Kharade
 *                      Tanmay Kothale 
 *                      Vishal Raj
 * 
 * @date            :   May 03, 2022
 * 
 * @references      :   1. https://github.com/powergee/TrafficLightDetection
 *                      2. http://mercury.pr.erau.edu/~siewerts/cec450/code/sequencer_generic/seqgen.c
 * 
 */
 
/**********************************************************************/
/*                          LIBRARY FILES                             */
/**********************************************************************/
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <time.h>
#include "process.h"
#include "capture.h"

/**********************************************************************/
/*               NAMESPACE FOR IO OPERATIONS IN CPP                   */
/**********************************************************************/
using namespace std;

/**********************************************************************/
/*                  EXTERN GLOBAL VARIABLES                           */
/**********************************************************************/
cv::Mat frame;
bool frame_flag = false;

/*see documentation in capture.h*/
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
{
      int dt_sec=stop->tv_sec - start->tv_sec;
      int dt_nsec=stop->tv_nsec - start->tv_nsec;


      // case 1 - less than a second of change
      if(dt_sec == 0)
      {

         if(dt_nsec >= 0 && dt_nsec < NSEC_PER_SEC)
         {
             //printf("nanosec greater at stop than start\n");
             delta_t->tv_sec = 0;
             delta_t->tv_nsec = dt_nsec;
         }

         else if(dt_nsec > NSEC_PER_SEC)
         {
                 //printf("nanosec overflow\n");
         delta_t->tv_sec = 1;
         delta_t->tv_nsec = dt_nsec-NSEC_PER_SEC;
         }

         else // dt_nsec < 0 means stop is earlier than start
         {
            printf("stop is earlier than start\n");
            return(ERROR);  
         }
      }

      // case 2 - more than a second of change, check for roll-over
      else if(dt_sec > 0)
      {
        
        //printf("dt more than 1 second\n");
         if(dt_nsec >= 0 && dt_nsec < NSEC_PER_SEC)
         {
             //printf("nanosec greater at stop than start\n");
             delta_t->tv_sec = dt_sec;
             delta_t->tv_nsec = dt_nsec;
         }

         else if(dt_nsec > NSEC_PER_SEC)
         {
             //printf("nanosec overflow\n");
             delta_t->tv_sec = delta_t->tv_sec + 1;
             delta_t->tv_nsec = dt_nsec-NSEC_PER_SEC;
         }

         else // dt_nsec < 0 means roll over
         {
            //printf("nanosec roll over\n");
            delta_t->tv_sec = dt_sec-1;
            delta_t->tv_nsec = NSEC_PER_SEC + dt_nsec;
         }
      }

      return(OK);
}

/*see documentation in capture.h*/
void capture_frame(cv::VideoCapture cap)
{
    
    frame_flag = cap.read(frame);
    if (!frame_flag)
    {
        cout << "Frame capture failed! Entering infinite loop...." << endl;
        while(1);
    }
}
