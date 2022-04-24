#ifndef _CAPTURE_H_
#define _CAPTURE_H_

#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <time.h>
#include "process.h"

#define ERROR (-1)
#define OK (0)
#define NSEC_PER_SEC (1000000000)
#define NSEC_PER_MSEC (1000000)
#define NSEC_PER_MICROSEC (1000)
#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_SEC (1000000000)
#define TRUE (1)
#define FALSE (0)

extern cv::Mat frame;
extern bool frame_flag; 

void capture_frame(cv::VideoCapture cap);
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t);

#endif /*_CAPTURE_HPP_*/
