/*
 * @file_name       :   main.cpp
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
 *                      2. http://mercury.pr.erau.edu/~siewerts/cec450/code/sequencer_generic/seqgen.c
 * 
 */

// This is necessary for CPU affinity macros in Linux
#define _GNU_SOURCE

/**********************************************************************/
/*                          LIBRARY FILES                             */
/**********************************************************************/
#include <stdio.h>
#include <stdint.h>         
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>        
#include <sched.h>          
#include <sys/sysinfo.h>   
#include <time.h>     
#include <sys/time.h>
#include <errno.h>      
#include <semaphore.h>    
#include <signal.h>        
#include <iostream>         
#include <sstream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include "capture.h"
#include "process.h"
#include "uart.h"
#include "calculate.h"
#include "udm.h"

/**********************************************************************/
/*               NAMESPACE FOR IO OPERATIONS IN CPP                   */
/**********************************************************************/
using namespace std;

/**********************************************************************/
/*                  PRIVATE MACROS AND DEFINES                        */
/**********************************************************************/
#define TRUE (1)
#define FALSE (0)
#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (1)
#define NUM_THREADS (7+1)
#define SCHED_POLICY SCHED_FIFO
#define FIXED_DEADLINE (400.00)

void *Sequencer(void *threadp);
void *Service_1(void *threadp);
void *Service_2(void *threadp);
void *Service_3(void *threadp);
void *Service_4(void *threadp);
void *Service_5(void *threadp);
 
static void print_scheduler(void);

/**********************************************************************/
/*              TYPEDEFS, DATA STRUCTURES & ENUMERATIONS              */
/**********************************************************************/
typedef void* (*worker_t)(void*);
// For sched fifo this ranges from 0-99
typedef uint8_t priority_t;

//for thread callback function
typedef struct
{
    int threadIdx;
    worker_t worker;
} threadParams_t;

//enum for number of services
typedef enum service
{
    SCHEDULER,
    SERVICE_1,
    SERVICE_2,
    SERVICE_3,
    SERVICE_4,
    SERVICE_5,
    NUM_SERVICES
}service_t;

//to compute wcet and avg execution time
typedef struct service_params_s
{
    uint32_t wcet;
    uint32_t total_ci;
    double avg_ci;
    
}service_params_t;

/**********************************************************************/
/*                          GLOBAL VARIABLES                          */
/**********************************************************************/
priority_t rt_max_prio, rt_min_prio;                //to set priorities of tasks
pthread_t threads[NUM_THREADS];                     //array of threads
threadParams_t threadParams[NUM_THREADS];           //array to store thread parameters
static struct sched_param rt_param[NUM_THREADS];    //array to store scheduler parameters
pthread_attr_t rt_sched_attr[NUM_THREADS];          //array to store scheduler attributes

sem_t semS1, semS2, semS3, semS4, semS5;            //semaphores for synchronization

struct timeval start_time_val;                      //to set scheduler frequency

//following structure instances are used for logging purposes
struct timespec start_s1 = {0,0}, start_s2 = {0,0}, start_s3 = {0,0}, start_s4 = {0,0}, start_total = {0,0},
                end_s1 = {0,0}  , end_s2 = {0,0}  , end_s3 = {0,0}  , end_s4 = {0,0}  , end_total = {0,0},
                delta_s1 = {0,0}, delta_s2 = {0,0}, delta_s3 = {0,0}, delta_s4 = {0,0}, delta_total = {0,0};

//array of service parameter structure to store wcet and acet
service_params_t service_times[NUM_SERVICES];

//global object used to capture frames
cv::VideoCapture cap(0); 

//to find contours in a given area of a frame
int min_area, max_area;

//to calculate deadlines and distance
float distance_cm = 0, computed_deadline = 0, time_to_stop_sec = 0;
double distance_udm;
uint32_t ctr=0;

/************************************************************************************
 * @brief   :   Function to print the scheduler properties
 *              
 * @param   :   none
 *
 * @return  :   void
 *              
*************************************************************************************/
static void print_scheduler(void)
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
       case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
       case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n"); exit(-1);
         break;
       case SCHED_RR:
           printf("Pthread Policy is SCHED_RR\n"); exit(-1);
           break;
       default:
           printf("Pthread Policy is UNKNOWN\n"); exit(-1);
   }
}

/************************************************************************************
 * @brief   :   Function to get the priority of the service
 *              
 * @param   :   service_t   -  service who's priority is to be returned
 *
 * @return  :   priority_t  -  priority of service
 *              
*************************************************************************************/
priority_t get_priority(service_t t)
{
    switch (t)
    {
        case SCHEDULER:
            return rt_max_prio;
            break;
        
        case SERVICE_1:
            return rt_max_prio - 1;
            break;

        case SERVICE_2:
            return rt_max_prio - 2;
            break;

        case SERVICE_3:
            return rt_max_prio - 3; 
            break;
            
        case SERVICE_4:
            return rt_max_prio - 4;
            break;
            
        case SERVICE_5:
            return rt_max_prio - 5; 
            break;
            
        default:
            printf("ERROR: No such task\n");
            return -1;
            break;
    }
}

/************************************************************************************
 * @brief   :   Function that assigns a worker function to each thread
 *              
 * @param   :   service_t t -  service who's worker needs to be assigned
 *
 * @return  :   worker_t    -  worker function for a particular service
 *              
*************************************************************************************/
worker_t get_worker(service_t t)
{
    switch (t)
    {
        case SCHEDULER:
            return Sequencer;
            break;
        
        case SERVICE_1:
            return Service_1;
            break;

        case SERVICE_2:
            return Service_2;
            break;

        case SERVICE_3:
            return Service_3;
            break;
        
        case SERVICE_4:
            return Service_4;
            break;

        case SERVICE_5:
            return Service_5; 
            break;
            
        default:
            cout << "Error: No such task found." << endl;
            return 0;
            break;
    }
}

/************************************************************************************
 * @brief   :   configures the scheduler to execute all services
 *              
 * @param   :   none
 *
 * @return  :   none
 *              
*************************************************************************************/
void configure_service_scheduler(void)
{
    cpu_set_t allcpuset;
    int i;
    pid_t mainpid;
    int rc, scope;

    struct sched_param main_param;
    pthread_attr_t main_attr;

    // Work related to CPU and cores
    cout << "System has " << get_nprocs_conf() << " processors configured and " << get_nprocs() << " available." << endl;

    CPU_ZERO(&allcpuset);
    
    for(i=0; i < NUM_CPU_CORES; i++)
        CPU_SET(i, &allcpuset);

    cout << "Using CPUS = "<< CPU_COUNT(&allcpuset) << " from total available." << endl;

    // Work related to services
    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    // Record the main application's process ID
    mainpid=getpid();    

    rc=sched_getparam(mainpid, &main_param);
    main_param.sched_priority=rt_max_prio;
    rc=sched_setscheduler(getpid(), SCHED_POLICY, &main_param);
    if(rc < 0) 
    {
        perror("main_param");
    }

    print_scheduler();

    pthread_attr_getscope(&main_attr, &scope);

    if(scope == PTHREAD_SCOPE_SYSTEM)
      cout << "PTHREAD SCOPE SYSTEM" << endl;
    else if (scope == PTHREAD_SCOPE_PROCESS)
      cout << "PTHREAD SCOPE PROCESS" << endl;
    else
      cout << "PTHREAD SCOPE UNKNOWN" << endl;

    cout << "rt_max_prio= " << rt_max_prio << endl;
    cout << "rt_min_prio= " << rt_min_prio << endl;
}

/************************************************************************************
 * @brief   :   schedules all the services
 *              
 * @param   :   threadp - Thread parameters 
 *
 * @return  :   none
 *              
*************************************************************************************/
void *Sequencer(void *threadp)
{
    struct timeval current_time_val;
    struct timespec delay_time = {0,10000000}; // delay for 10msec, Frequency = 100 Hz

    struct timespec remaining_time;
    double current_time;
    double residual;
    int rc, delay_cnt=0;
    unsigned long long seqCnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    gettimeofday(&current_time_val, (struct timezone *)0);
    
    cout << "Sequencer thread @ sec= " << (int)(current_time_val.tv_sec-start_time_val.tv_sec) 
         << ", msec= " << (int)current_time_val.tv_usec/USEC_PER_MSEC << endl;

    do
    {
        delay_cnt=0; residual=0.0;
        
        do
        {
            rc=nanosleep(&delay_time, &remaining_time);

            if(rc == EINTR)
            { 
                residual = remaining_time.tv_sec + ((double)remaining_time.tv_nsec / (double)NANOSEC_PER_SEC);

                if(residual > 0.0)
                    cout << "residual= " << residual << ", sec=" << (int)remaining_time.tv_sec << ", nsec=" << (int)remaining_time.tv_nsec << endl;
 
                delay_cnt++;
            }
            else if(rc < 0)
            {
                perror("Sequencer nanosleep");
                exit(-1);
            }
           
        } while((residual > 0.0) && (delay_cnt < 100));

        seqCnt++;
        gettimeofday(&current_time_val, (struct timezone *)0);

        if(delay_cnt > 1) 
            cout << "Sequencer looping delay " << delay_cnt << endl;
            
        //post the first service every 450 msec = 2.22 Hz
        if((seqCnt % 45) == 0) 
        {
            sem_post(&semS1);
        }

    } while(1);

    pthread_exit((void *)0);
}

/************************************************************************************
 * @brief   :   Captures the frame
 *              
 * @param   :   threadp - Thread parameters 
 *
 * @return  :   none
 *              
*************************************************************************************/
void *Service_1(void *threadp)
{

    while(1)
    {
        sem_wait(&semS1);                               //wait for sequencer to post the semaphore

        clock_gettime(CLOCK_REALTIME, &start_total);    //record start time of total execution
        clock_gettime(CLOCK_REALTIME, &start_s1);       //record start time for this service
        capture_frame(cap);                             //capture the frame
        clock_gettime(CLOCK_REALTIME, &end_s1);         //record end time for this service
        
        sem_post(&semS2);                               //post semaphore for next service
    }

    pthread_exit((void *)0);
}

/************************************************************************************
 * @brief   :   Performs UART transmission and reception
 *              
 * @param   :   none
 *
 * @return  :   none
 *              
*************************************************************************************/
void test(void)
{
    int fd;
    int rv = 0;
    char s = getColor();
    
    /*Override Image processing algorithm*/
    if(distance_udm < 35.0)
        s = RED;
    
    char r;
    
    fd = open("/dev/ttyS0", O_RDWR);
    if (fd == -1)
        perror("error: open");
        
    rv = write(fd, &s, 1);
    if (rv == -1)
        perror("error: write");
    
    rv = read(fd, &r, 1);
    if (rv == -1)
        perror("error: read");
    
    close(fd);

}

/************************************************************************************
 * @brief   :   Processes the image and checks for traffic light
 *              
 * @param   :   threadp - Thread parameters 
 *
 * @return  :   none
 *              
*************************************************************************************/
void *Service_2(void *threadp)
{

    while(1)
    {   
        sem_wait(&semS2);

        clock_gettime(CLOCK_REALTIME, &start_s2);
        process_image(min_area, max_area); 
        clock_gettime(CLOCK_REALTIME, &end_s2);
        
        sem_post(&semS3);
    }

    pthread_exit((void *)0);
}

/************************************************************************************
 * @brief   :   Measures distance from Ultrasonic sensor
 *              
 * @param   :   threadp - Thread parameters 
 *
 * @return  :   none
 *              
*************************************************************************************/
void *Service_3(void *threadp)
{

    while(1)
    {
        sem_wait(&semS3);
        
        //changes by Tanmay
        clock_gettime(CLOCK_REALTIME, &start_s3);
        
        /*UDM Test*/
        distance_udm = get_distance();

        clock_gettime(CLOCK_REALTIME, &end_s3);
        
        sem_post(&semS4);
    }

    pthread_exit((void *)0);
}

/************************************************************************************
 * @brief   :   Sends the data packet to TIVA using UART
 *              
 * @param   :   threadp - Thread parameters 
 *
 * @return  :   none
 *              
*************************************************************************************/
void *Service_4(void *threadp)
{
    char c;
    while(1)
    {   
        sem_wait(&semS4);

        clock_gettime(CLOCK_REALTIME, &start_s4);
        test();
        clock_gettime(CLOCK_REALTIME, &end_s4);
        
        clock_gettime(CLOCK_REALTIME, &end_total);
        
        sem_post(&semS5);
    }

    pthread_exit((void *)0);
}

/************************************************************************************
 * @brief   :   Logs the timings for all services
 *              
 * @param   :   threadp - Thread parameters 
 *
 * @return  :   none
 *              
*************************************************************************************/
void *Service_5(void *threadp)
{
    static uint32_t total_time_msec;
    char c;
    
    uint32_t s1_msec=0, s2_msec=0, s3_msec=0, s4_msec=0;
    
    while(1)
    {   
        sem_wait(&semS5);
        
        ctr++;    
        
        delta_t(&end_s1, &start_s1, &delta_s1);
        
        delta_t(&end_s2, &start_s2, &delta_s2);
        
        delta_t(&end_s3, &start_s3, &delta_s3);

        delta_t(&end_s4, &start_s4, &delta_s4);

        delta_t(&end_total, &start_total, &delta_total);
        
        s1_msec = delta_s1.tv_nsec/NSEC_PER_MSEC;
        s2_msec = delta_s2.tv_nsec/NSEC_PER_MSEC;
        s3_msec = delta_s3.tv_nsec/NSEC_PER_MSEC;
        s4_msec = delta_s4.tv_nsec/NSEC_PER_MSEC;
        

        service_times[SERVICE_1].total_ci += s1_msec;
        service_times[SERVICE_2].total_ci += s2_msec;
        service_times[SERVICE_3].total_ci += s3_msec;
        service_times[SERVICE_4].total_ci += s4_msec; 
        
        if(s1_msec > service_times[SERVICE_1].wcet)
        {
            service_times[SERVICE_1].wcet = s1_msec;
        }
        
        if(s2_msec > service_times[SERVICE_2].wcet)
        {
            service_times[SERVICE_2].wcet = s2_msec;
        }
        
        if(s3_msec > service_times[SERVICE_3].wcet)
        {
            service_times[SERVICE_3].wcet = s3_msec;
        }
        
        if(s4_msec > service_times[SERVICE_4].wcet)
        {
            service_times[SERVICE_4].wcet = s4_msec;
        }
        
        cout << "Distance: " << distance_udm << " cm" << endl;
        
        cout << "Service 1: Frame Capture: " << delta_s1.tv_sec << " sec " 
             << s1_msec << " msec" 
             << endl;
             
        cout << "Service 2: Process Image: " << delta_s2.tv_sec << " sec " 
             << s2_msec << " msec" 
             << endl;
             
        cout << "Service 3: Measure Distance: " << delta_s3.tv_sec << " sec " 
             << s3_msec << " msec" 
             << endl;
             
        cout << "Service 4: Send Data: " << delta_s4.tv_sec << " sec " 
             << s4_msec << " msec" 
             << endl;
        
        cout << "Total Time: " << delta_total.tv_sec << " sec " 
             << delta_total.tv_nsec/NSEC_PER_MSEC << " msec" 
             << endl;
             
        total_time_msec = (uint32_t)((delta_total.tv_sec*1000) + (delta_total.tv_nsec/NSEC_PER_MSEC));
        
        if (total_time_msec < FIXED_DEADLINE)
        {
            cout << "Deadline Met!" << endl;
        }
        else
        {
            cout << "Deadline missed!" << endl;
        }
        
        cout << "-------------------------------------------------------" << endl;
             
    }

    pthread_exit((void *)0);
}

/************************************************************************************
 * @brief   :   Configure all services as threads and assign a worker function
 *              
 * @param   :   none
 *
 * @return  :   none
 *              
*************************************************************************************/
static void configure_services(void)
{
    int i, rc;
    cpu_set_t threadcpu;

    for(i=0; i < NUM_SERVICES; i++)
    {

        CPU_ZERO(&threadcpu);
        CPU_SET(3, &threadcpu);

        rc=pthread_attr_init(&rt_sched_attr[i]);
        rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
        rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);


        rt_param[i].sched_priority = get_priority((service_t)i); 
        pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

        threadParams[i].threadIdx = i;
        threadParams[i].worker = get_worker((service_t)i);

        rc=pthread_create(&threads[i],              // pointer to thread descriptor
                        &rt_sched_attr[i],          // use specific attributes
                        threadParams[i].worker,     // thread function entry point
                        (void *)&(threadParams[i])  // parameters to pass in
                        );
        if(rc < 0)
            printf("pthread_create for service %d\n", i);
        else
            printf("pthread_create successful for service %d\n", i);

    }
   
    printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));

}

/************************************************************************************
 * @brief   :   Initialize all semaphores for synchronization
 *              
 * @param   :   none
 *
 * @return  :   none
 *              
*************************************************************************************/
void semaphores_init()
{
    if (sem_init (&semS1, 0, 0)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }
    if (sem_init (&semS2, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
    if (sem_init (&semS3, 0, 0)) { printf ("Failed to initialize S3 semaphore\n"); exit (-1); }
    if (sem_init (&semS4, 0, 0)) { printf ("Failed to initialize S4 semaphore\n"); exit (-1); }
    if (sem_init (&semS5, 0, 0)) { printf ("Failed to initialize S5 semaphore\n"); exit (-1); }
       
}

/************************************************************************************
 * @brief   :   initialize the camera
 *              
 * @param   :   none
 *
 * @return  :   none
 *              
*************************************************************************************/
void camera_init()
{
    //cv::VideoCapture cap(0);
    double width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    cv::namedWindow("Result");
    
    //int min_area, max_area;
    cv::createTrackbar("Minimum Area", "Result", &min_area, 100000);
    cv::createTrackbar("Maximum Area", "Result", &max_area, 100000);
    cv::setTrackbarPos("Minimum Area", "Result", 1000);
    cv::setTrackbarPos("Maximum Area", "Result", 100000);
}

/************************************************************************************
 * @brief   :   handles signal (in this case, SIGTERM)
 *              
 * @param   :   sig_no - macro defined for a specific signal
 *
 * @return  :   none
 *              
*************************************************************************************/
void sighandler(int sig_no)
{

    cout<<"SIGTERM detected!"<<endl;
    
    gpioTerminate();
    
    cout << "Average execution times: " << endl;
    
    cout << "For S1: " << service_times[SERVICE_1].total_ci/ctr << endl;
    cout << "For S2: " << service_times[SERVICE_2].total_ci/ctr << endl;
    cout << "For S3: " << service_times[SERVICE_3].total_ci/ctr << endl;
    cout << "For S4: " << service_times[SERVICE_4].total_ci/ctr << endl;
    
        
    cout << "\nWorst Case Execution Time: " << endl;
    
    cout << "For S1: " << service_times[SERVICE_1].wcet << endl;
    cout << "For S2: " << service_times[SERVICE_2].wcet << endl;
    cout << "For S3: " << service_times[SERVICE_3].wcet << endl;
    cout << "For S4: " << service_times[SERVICE_4].wcet << endl;
    
    exit(0);
}

/************************************************************************************
 * @brief   :   initializes ultrasonic sensor
 *              
 * @param   :   none
 *
 * @return  :   none
 *              
*************************************************************************************/
void udm_init()
{
        if(gpioInitialise() < 0){
            cout << "pigpio initialisation failed" << endl; 
            signal(SIGINT, sighandler); 
            exit(1);
        }
        else{
            signal(SIGINT, sighandler); 
            
            cout << "pigpio initialisation ok" << endl;

            init_udm();
        }
}

/************************************************************************************
 * @brief   :   application entry point
 *              
 * @param   :   argc - number of command line arguments
 *              argv - command line argument as string
 *
 * @return  :   zero
 *              
*************************************************************************************/
int main(int argc, char** argv)
{
    if (argc < 2)
    {
        cout << "Please provide distance. Do sudo ./main <distance_in_cm>" << endl;
        cout << "Insufficient arguments. Exiting..." << endl;
        exit (EXIT_FAILURE);
    }
    else
    {
        sscanf(argv[1], "%f", &distance_cm);
    }
    
    cout << "-----------------------------------------------------------------------" << endl;
    
    cout << "Assuming red light is detected " << distance_cm << " cm from Car's current position:" << endl;
    cout << "Current speed of car: " << CURRENT_SPEED_OF_CAR << " cm/sec" << endl;
    time_to_stop_sec = time_to_stop_in_sec (CURRENT_SPEED_OF_CAR, distance_cm);
    cout << "Total time required for car to come to a full Stop: " << time_to_stop_sec << " seconds" << endl;
    
    computed_deadline = compute_deadline_to_complete_tasks (distance_cm);
    cout << "Computed deadline for all tasks: " << computed_deadline/2 << " seconds" << endl;
    cout << "-----------------------------------------------------------------------" << endl;
    
    int i;

    camera_init(); 
    
    udm_init();
    
    configure_service_scheduler();

    configure_services();
    
    //should never come here
    for(i=0;i<NUM_SERVICES;i++)
    {
        pthread_join(threads[i], NULL);
    }

   return 0;
}
