#ifndef _CAPTURE_H_
#define _CAPTURE_H_

#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <time.h>
#include "process.h"

extern cv::Mat frame;
extern bool frame_flag; 

void capture_frame(cv::VideoCapture cap);
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t);

#endif /*_CAPTURE_HPP_*/
