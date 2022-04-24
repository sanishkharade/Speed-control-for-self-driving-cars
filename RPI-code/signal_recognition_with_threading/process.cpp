#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>

#include "process.h"
#include "capture.h"



color_t color = NONE;

using namespace std;

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

bool isConvex(Contour& c, double area)
{
    Contour hull_cntr;
    cv::convexHull(c, hull_cntr);
    double hull_area = cv::moments(hull_cntr).m00;
    return abs(hull_area - area) / area <= 0.1;
}

Shape labelPolygon(Contour& c, double area)
{
    double peri = cv::arcLength(c, true);
    Contour approx;
    cv::approxPolyDP(c, approx, 0.02*peri, true);

    if ((int)approx.size() == 7)
    {
        cv::Point center = std::accumulate(approx.begin(), approx.end(), cv::Point(0, 0)) / 7;
        int left_cnt, right_cnt;
        left_cnt = right_cnt = 0;

        for (int i = 0; i < 7; ++i)
        {
            if ((approx[i] - center).x >= 0)
                ++right_cnt;
            else
                ++left_cnt;
        }

        if (left_cnt > right_cnt)
            return Shape::Left;
        else
            return Shape::Right;
    }

    if (approx.size() > 7 && isConvex(c, area))
        return Shape::Circle;
    
    return Shape::Undefined;
}

std::string shapeToString(Shape s)
{
    switch (s)
    {
    case Shape::Circle:
        return "Circle";
    
    case Shape::Left:
        return "Left";

    case Shape::Right:
        return "Right";

    case Shape::Undefined:
        return "Undefined";
    }

    return "Error";
}

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
color_t getColor(void)
{
    return color;
}
void process_image(int min_area, int max_area)
{
	 struct timespec start_process, end_process;
     struct timespec time_dt_process;
    
     start_process   = {0,0};
     end_process     = {0,0};
     time_dt_process = {0,0};
	
	 clock_gettime(CLOCK_REALTIME, &start_process);
     cv::Mat redMasked = mask_img(frame, 0, 15, 180, 128);
     cv::Mat yellowMasked = mask_img(frame, 30, 15, 120, 60);
     cv::Mat greenMasked = mask_img(frame, 60, 15, 90, 60);
     //cv::Mat greenInverse = 255 - greenMasked;

     //cv::imshow("Found Red", redMasked);
     //cv::imshow("Found Yellow", yellowMasked);
     //cv::imshow("Found Green", greenMasked);
     //cv::imshow("Found Green (Inverse)", greenInverse);

     const static std::string captions[] = { "Red Light!", "Yellow Light!", "Green Light!"};//, "Left Direction!", "Right Direction!" };
     const static cv::Scalar colors[] = { cv::Scalar(0, 0, 255), cv::Scalar(131, 232, 252), cv::Scalar(0, 255, 0)};//, cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 0) };

     std::vector<Contour> found[] = {
         findShapes(Shape::Circle, redMasked, min_area, max_area),
         findShapes(Shape::Circle, yellowMasked, min_area, max_area),
         findShapes(Shape::Circle, greenMasked, min_area, max_area),
         //findShapes(Shape::Left, greenInverse, min_area, max_area),
         //findShapes(Shape::Right, greenInverse, min_area, max_area)
     };
        int i;
     for ( i= 0; i < 3; ++i)
     {
         if (!found[i].empty())
         {
             //cv::drawContours(frame, found[i], -1, colors[i], 2);
             //putTextAtCenter(frame, captions[i], colors[i]);
             clock_gettime(CLOCK_REALTIME, &end_process);
             delta_t(&end_process, &start_process, &time_dt_process);
                    
             cout << "Process Time: " << time_dt_process.tv_sec << "sec " 
                  <<time_dt_process.tv_nsec/NSEC_PER_MSEC << "msec" 
                  << endl;
                    
             cout << captions[i] << endl;
             setColor(i+1);
             break;
         }
     }
    if(i == 3)
    {
        setColor(0);
    }
     //cv::imshow("Result", frame); 
     
     //int key = cv::waitKey(1) & 0xff;
}
