#ifndef _PROCESS_H_
#define _PROCESS_H_

#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include "capture.h"

enum Shape { Circle, Left, Right, Undefined };
typedef std::vector<cv::Point> Contour;

cv::Mat mask_img(cv::Mat& frame, int h, int error, int s_min, int v_min);
std::vector<Contour> findShapes(Shape shapeToFind, cv::Mat& grey, int min_area, int max_area);
std::string shapeToString(Shape s);
Shape labelPolygon(Contour& c, double area);
bool isConvex(Contour& c, double area);
void process_image(int min_area, int max_area);

typedef enum
{
	NONE,
    RED,
    YELLOW,
    GREEN,
    
}color_t;

void setColor(int c);
color_t getColor(void);
#endif /*_PROCESS_HPP_*/
