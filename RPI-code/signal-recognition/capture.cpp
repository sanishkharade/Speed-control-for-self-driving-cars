#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <time.h>
#include "process.h"
#include "capture.h"

#define ERROR (-1)
#define OK (0)
#define NSEC_PER_SEC (1000000000)
#define NSEC_PER_MSEC (1000000)
#define NSEC_PER_MICROSEC (1000)
#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_SEC (1000000000)
#define TRUE (1)
#define FALSE (0)

using namespace std;

cv::Mat frame;

bool frame_flag = false;

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

void capture_frame(cv::VideoCapture cap)
{
    struct timespec start_frame, end_frame;
    struct timespec time_dt_frame;
    
    start_frame     = {0,0};
    end_frame       = {0,0};
    time_dt_frame   = {0,0};
    
    clock_gettime(CLOCK_REALTIME, &start_frame);
    frame_flag = cap.read(frame);
    if (frame_flag)
    {
        clock_gettime(CLOCK_REALTIME, &end_frame);
        delta_t(&end_frame, &start_frame, &time_dt_frame);
            
        cout << "Frame Capture Time: " << time_dt_frame.tv_sec << "sec " 
             <<time_dt_frame.tv_nsec/1000000 << "msec" 
             << endl;
    }
    else
    {
        cout << "Frame capture failed! Entering infinite loop...." << endl;
        while(1);
    }
}
