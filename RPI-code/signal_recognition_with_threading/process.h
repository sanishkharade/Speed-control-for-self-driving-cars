/*
 * @file_name       :   process.h
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
#ifndef _PROCESS_H_
#define _PROCESS_H_

/**********************************************************************/
/*                          LIBRARY FILES                             */
/**********************************************************************/
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include "capture.h"

/*ENUMERATION OF DESIRED SHAPES*/
enum Shape { Circle, Undefined };
/*TYPEDEF FOR TEMPLATE*/
typedef std::vector<cv::Point> Contour;

typedef enum
{
    NONE,
    RED,
    YELLOW,
    GREEN,
    
}color_t;


/************************************************************************************
 * @brief   :   masks the image from BGR to HSV values
 *              
 * @param   :   frame	-	captured frame to be masked
 * 		h	-	Hue value
 * 		error	-	error margin acceptable
 * 		s_min	-	minimum value for saturation
 * 		v_min	-	minimum value for vignette
 * 
 *
 * @return  :   masked image
 *              
*************************************************************************************/
cv::Mat mask_img(cv::Mat& frame, int h, int error, int s_min, int v_min);

/************************************************************************************
 * @brief   :   performs contour detection to identify various shapes (circles)
 *              
 * @param   :   shapeToFind	-	shape that needs to be detected
 * 		grey		-	masked image
 * 		min_area	-	lower limit of area to look for
 * 		max_area	-	upper limit of area to look for
 *
 * @return  :   identified shape
 *              
*************************************************************************************/
std::vector<Contour> findShapes(Shape shapeToFind, cv::Mat& grey, int min_area, int max_area);

/************************************************************************************
 * @brief   :   Lables the polygon identified in the contour detection
 *              
 * @param   :   c	-	contour
 * 		area	-	area to look for
 *
 * @return  :   identified shape
 *              
*************************************************************************************/
Shape labelPolygon(Contour& c, double area);

/************************************************************************************
 * @brief   :   identifies circles (convex contours)
 *              
 * @param   :   c	-	contour
 * 		area	-	area to look for
 *
 * @return  :   true if circle is detected, false otherwise
 *              
*************************************************************************************/
bool isConvex(Contour& c, double area);

/************************************************************************************
 * @brief   :   processes the frame captured
 *              
 * @param   :   min_area	-	lower limit of area to look for
 * 		max_area	-	upper limit of area to look for
 *
 * @return  :   none
 *              
*************************************************************************************/
void process_image(int min_area, int max_area);

/************************************************************************************
 * @brief   :   sets the color once identified
 *              
 * @param   :   c	-	macro of color identified
 *
 * @return  :   none
 *              
*************************************************************************************/
void setColor(int c);

/************************************************************************************
 * @brief   :   determines which color was identified and returns it to calling function
 *              
 * @param   :   none
 *
 * @return  :   color identified
 *              
*************************************************************************************/
color_t getColor(void);
#endif /*_PROCESS_H_*/
