#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <time.h>

#include "process.h"
#include "capture.h"

int main()
{
    cv::VideoCapture cap(0);
    double width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    cv::namedWindow("Result");
    
    int min_area, max_area;
    cv::createTrackbar("Minimum Area", "Result", &min_area, 100000);
    cv::createTrackbar("Maximum Area", "Result", &max_area, 100000);
    cv::setTrackbarPos("Minimum Area", "Result", 1000);
    cv::setTrackbarPos("Maximum Area", "Result", 100000);
    
    while(1)
    {
		capture_frame(cap);
		process_image(min_area, max_area);
	}
}
