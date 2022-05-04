/*
 * @file_name       :   udm.h
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

#ifndef _UDM_H_
#define _UDM_H_

//function prototypes
/************************************************************************************
 * @brief   :   calculates distance using ultrasonic sensor
 *              
 * @param   :   none
 *
 * @return  :   distance	-	calculated distance
 *              
*************************************************************************************/
double get_distance();

/************************************************************************************
 * @brief   :   initialize ultrasonic function
 *              
 * @param   :   none
 *
 * @return  :   none
*************************************************************************************/
void init_udm();


#endif /*_UDM_H_*/



