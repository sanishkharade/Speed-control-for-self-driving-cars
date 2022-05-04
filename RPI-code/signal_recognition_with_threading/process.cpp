/*
 * @file_name       :   process.cpp
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
 * 
 */
 
/**********************************************************************/
/*                          LIBRARY FILES                             */
/**********************************************************************/
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include "process.h"
#include "capture.h"

/**********************************************************************/
/*                          GLOBAL VARIABLES                          */
/**********************************************************************/
color_t color = NONE;

/**********************************************************************/
/*               NAMESPACE FOR IO OPERATIONS IN CPP                   */
/**********************************************************************/
using namespace std;

/*see documentation in process.h*/
cv::Mat mask_img(cv::Mat& frame, int h, int error, int s_min, int v_min)
{
    cv::Mat hsv_img;
    cv::cvtColor(frame, hsv_img, cv::COLOR_BGR2HSV);
    int lowH = (h-error >= 0) ? h-error : h-error+180;
    int highH = (h+error <= 180) ? h+error : h+error-180;

    std::vector<cv::Mat> channels;
    cv::split(hsv_img, channels);
    if (lowH < highH)
        cv::bitwise_and(lowH <= channels[0], channels[0] <= highH, channels[0]);
    else
        cv::bitwise_or(lowH <= channels[0], channels[0] <= highH, channels[0]);
    
    channels[1] = channels[1] > s_min;
    channels[2] = channels[2] > v_min;
    
    cv::Mat mask = channels[0];
    for (int i = 1; i < 3; ++i)
        cv::bitwise_and(channels[i], mask, mask);
    
    cv::Mat grey = cv::Mat::zeros(mask.rows, mask.cols, CV_8U);
    for (int row = 0; row < mask.rows; ++row)
    {
        for (int col = 0; col < mask.cols; ++col)
        {
            auto v1 = channels[0].data[row*mask.cols + col];
            auto v2 = channels[1].data[row*mask.cols + col];
            auto v3 = channels[2].data[row*mask.cols + col];
            grey.data[row*mask.cols + col] = (v1 && v2 && v3 ? 255 : 0);
        }
    }

    return grey;
}

/*see documentation in process.h*/
bool isConvex(Contour& c, double area)
{
    Contour hull_cntr;
    cv::convexHull(c, hull_cntr);
    double hull_area = cv::moments(hull_cntr).m00;
    return abs(hull_area - area) / area <= 0.1;
}

/*see documentation in process.h*/
Shape labelPolygon(Contour& c, double area)
{
    double peri = cv::arcLength(c, true);
    Contour approx;
    cv::approxPolyDP(c, approx, 0.02*peri, true);

    if (approx.size() > 7 && isConvex(c, area))
        return Shape::Circle;
    
    return Shape::Undefined;
}

/*see documentation in process.h*/
std::vector<Contour> findShapes(Shape shapeToFind, cv::Mat& grey, int min_area, int max_area)
{
    std::vector<Contour> contours;
    cv::findContours(grey, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    std::vector<Contour> found;
    
    for (auto c : contours)
    {
        cv::Moments m;
        m = cv::moments(c);
        
        if (m.m00 != 0 && min_area <= m.m00 && m.m00 <= max_area)
        {
            Shape shape = labelPolygon(c, m.m00);
            if (shape == shapeToFind)
                found.push_back(c);
        }
    }

    return found;
}

/*see documentation in process.h*/
void setColor(int c)
{
    
    switch(c)
    {
        case 1:
            color = RED;
            break;
            
        case 2:
            color = YELLOW;
            break;         
        
        case 3:
            color = GREEN;
            break;
            
        default:
            color = NONE;
            break;
    }
}

/*see documentation in process.h*/
color_t getColor(void)
{
    return color;
}

/*see documentation in process.h*/
void process_image(int min_area, int max_area)
{    
     cv::Mat redMasked = mask_img(frame, 0, 15, 180, 128);
     cv::Mat yellowMasked = mask_img(frame, 30, 15, 120, 60);
     cv::Mat greenMasked = mask_img(frame, 60, 15, 90, 60);

     const static std::string captions[] = { "Red Light!", "Yellow Light!", "Green Light!"};
     const static cv::Scalar colors[] = { cv::Scalar(0, 0, 255), cv::Scalar(131, 232, 252), cv::Scalar(0, 255, 0)};

     std::vector<Contour> found[] = {
         findShapes(Shape::Circle, redMasked, min_area, max_area),
         findShapes(Shape::Circle, yellowMasked, min_area, max_area),
         findShapes(Shape::Circle, greenMasked, min_area, max_area),
     };
    
     int i;
     for ( i= 0; i < 3; ++i)
     {
         if (!found[i].empty())
         {                    
             cout << captions[i] << endl;
             setColor(i+1);
             break;
         }
     }
    if(i == 3)
    {
        setColor(0);
    }
}
