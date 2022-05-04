/*
 * @file_name       :   capture.h
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

#ifndef _CAPTURE_H_
#define _CAPTURE_H_

/**********************************************************************/
/*                          LIBRARY FILES                             */
/**********************************************************************/
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <time.h>
#include "process.h"

/**********************************************************************/
/*                  PRIVATE MACROS AND DEFINES                        */
/**********************************************************************/
#define ERROR (-1)
#define OK (0)
#define NSEC_PER_SEC (1000000000)
#define NSEC_PER_MSEC (1000000)
#define NSEC_PER_MICROSEC (1000)
#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_SEC (1000000000)
#define TRUE (1)
#define FALSE (0)

/**********************************************************************/
/*                  EXTERN GLOBAL VARIABLES                           */
/**********************************************************************/
//defined in capture.cpp
extern cv::Mat frame; 
extern bool frame_flag; 

/************************************************************************************
 * @brief   :   captures a frame from camera
 *              
 * @param   :   cv::VideoCapture cap - global object instance used to capture frame
 *
 * @return  :   none
 *              
*************************************************************************************/
void capture_frame(cv::VideoCapture cap);

/************************************************************************************
 * @brief   :   calculates absolute difference between two timespec structures
 *              
 * @param   :   stop 	- end time
 * 				start 	- start time
 * 				delta_t - structure in which absolute difference will be stored
 *
 * @return  :   0 on success, -1 on failure
 *              
*************************************************************************************/
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t);

#endif /*_CAPTURE_H_*/
