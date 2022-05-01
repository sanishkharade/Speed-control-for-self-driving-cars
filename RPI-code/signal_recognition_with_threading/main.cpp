// This is necessary for CPU affinity macros in Linux
#define _GNU_SOURCE

#include <stdio.h>
#include <stdint.h>         // for uint8_t
#include <stdlib.h>
#include <unistd.h>

#include <pthread.h>        // for pthreads
#include <sched.h>          // for cpu functions

#include <sys/sysinfo.h>    // for get_nprocs

#include <time.h>           // for time function
#include <sys/time.h>

#include <errno.h>          // for error tracking
#include <semaphore.h>      // for semaphores

//includes for OpenCV -- changed by Tanmay
#include "capture.h"
#include "process.h"
#include "uart.h"
#include "calculate.h"
#include <iostream>
#include <sstream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include "udm.h"
#include <signal.h>

using namespace std;

#define TRUE (1)
#define FALSE (0)
#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (1)
#define NUM_THREADS (7+1)
#define SCHED_POLICY SCHED_FIFO

// For sched fifo this ranges from 0-99
typedef uint8_t priority_t;


void *Sequencer(void *threadp);

void *Service_1(void *threadp);
void *Service_2(void *threadp);
void *Service_3(void *threadp);
void *Service_4(void *threadp);
void *Service_5(void *threadp);

static void print_scheduler(void);

typedef void* (*worker_t)(void*);
typedef struct
{
    int threadIdx;
    worker_t worker;
} threadParams_t;


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

priority_t rt_max_prio, rt_min_prio;

pthread_t threads[NUM_THREADS];
threadParams_t threadParams[NUM_THREADS];

static struct sched_param rt_param[NUM_THREADS];
pthread_attr_t rt_sched_attr[NUM_THREADS];

sem_t semS1, semS2, semS3, semS4, semS5;
struct timeval start_time_val;

struct timespec start_s1 = {0,0}, start_s2 = {0,0}, start_s3 = {0,0}, start_s4 = {0,0}, start_total = {0,0},
                end_s1 = {0,0}  , end_s2 = {0,0}  , end_s3 = {0,0}  , end_s4 = {0,0}  , end_total = {0,0},
                delta_s1 = {0,0}, delta_s2 = {0,0}, delta_s3 = {0,0}, delta_s4 = {0,0}, delta_total = {0,0};

typedef struct service_params_s
{
    uint32_t wcet;
    uint32_t total_ci;
    double avg_ci;
    
}service_params_t;

service_params_t service_times[NUM_SERVICES];

//changes by tanmay
cv::VideoCapture cap(0); 
int min_area, max_area;

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
            //return 
            break;
        
        case SERVICE_1:
            return rt_max_prio - 1;
            //return 
            break;

        case SERVICE_2:
            return rt_max_prio - 2;
            //return 
            break;

        case SERVICE_3:
            return rt_max_prio - 3;
            //return 
            break;
            
        case SERVICE_4:
            return rt_max_prio - 4;
            //return 
            break;
            
        case SERVICE_5:
            return rt_max_prio - 5;
            //return 
            break;
            
        default:
            printf("ERROR: No such task\n");
            return -1;
            break;
    }
}
worker_t get_worker(service_t t)
{
    switch (t)
    {
        case SCHEDULER:
            return Sequencer;
            //return 
            break;
        
        case SERVICE_1:
            return Service_1;
            //return 
            break;

        case SERVICE_2:
            return Service_2;
            //return 
            break;

        case SERVICE_3:
            return Service_3;
            //return 
            break;
        
        case SERVICE_4:
            return Service_4;
            //return 
            break;

        case SERVICE_5:
            return Service_5;
            //return 
            break;
            
        default:
            printf("ERROR: No such task\n");
            return 0;
            break;
    }
}
void configure_service_scheduler(void)
{
    cpu_set_t allcpuset;
    int i;
    pid_t mainpid;
    int rc, scope;

    struct sched_param main_param;
    pthread_attr_t main_attr;

    // Work related to CPU and cores
    printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

    CPU_ZERO(&allcpuset);
    
    for(i=0; i < NUM_CPU_CORES; i++)
        CPU_SET(i, &allcpuset);

    printf("Using CPUS = %d from total available.\n", CPU_COUNT(&allcpuset));


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
      printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
      printf("PTHREAD SCOPE PROCESS\n");
    else
      printf("PTHREAD SCOPE UNKNOWN\n");

    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);
}
void *Sequencer(void *threadp)
{
    struct timeval current_time_val;
    struct timespec delay_time = {0,10000000}; // delay for 33.33 msec, 30 Hz For Q1
    // Comment above line and uncomment below line for Q3	
    // struct timespec delay_time = {0,333333}; // delay for 0.33 msec, 3000 Hz For Q3
    struct timespec remaining_time;
    double current_time;
    double residual;
    int rc, delay_cnt=0;
    unsigned long long seqCnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    gettimeofday(&current_time_val, (struct timezone *)0);
    //syslog(LOG_CRIT, "Sequencer thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Sequencer thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    do
    {
        delay_cnt=0; residual=0.0;
        //printf("Sequencer\n");
        //gettimeofday(&current_time_val, (struct timezone *)0);
        //printf("Sequencer thread prior to delay @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        do
        {
            rc=nanosleep(&delay_time, &remaining_time);

            if(rc == EINTR)
            { 
                residual = remaining_time.tv_sec + ((double)remaining_time.tv_nsec / (double)NANOSEC_PER_SEC);

                if(residual > 0.0) printf("residual=%lf, sec=%d, nsec=%d\n", residual, (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec);
 
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
        //printf("Sequencer cycle %llu @ sec=%d, msec=%d\n", seqCnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);


        if(delay_cnt > 1) printf("Sequencer looping delay %d\n", delay_cnt);


        // Release each service at a sub-rate of the generic sequencer rate

        // Servcie_1 = RT_MAX-1	@ 3 Hz
        if((seqCnt % 30) == 0) 
        {
            //clock_gettime(CLOCK_REALTIME, &start_total);
            sem_post(&semS1);
        }

    } while(1);

    pthread_exit((void *)0);
}

void *Service_1(void *threadp)
{

    while(1)
    {
        sem_wait(&semS1);
        
        //changes by Tanmay
        clock_gettime(CLOCK_REALTIME, &start_total);
        
        clock_gettime(CLOCK_REALTIME, &start_s1);
        /*UDM Test*/
        //distance_udm = get_distance();
        capture_frame(cap); 
        clock_gettime(CLOCK_REALTIME, &end_s1);
        
        sem_post(&semS2);
    }

    pthread_exit((void *)0);
}

void test(void)
{
    int fd;
    int rv = 0;
    char s = getColor();
    
    /*Overrid UDM*/
    if(distance_udm < 30.0)
        s = RED;
    
    char r;
    
    //cout << "Opening file" << endl;
    fd = open("/dev/ttyS0", O_RDWR);
    if (fd == -1)
        perror("error: open");
        
    //cout << "Starting to write" << endl;
    rv = write(fd, &s, 1);
    if (rv == -1)
        perror("error: write");
    
    //cout << "Bytes written: " << rv << endl;
    printf("sent = %d\n",s);
    rv = read(fd, &r, 1);
    if (rv == -1)
        perror("error: read");
    
    //cout << "Bytes read: " << rv << endl;
    printf("received = %d\n",r);
    close(fd);
    //cout << "s = " << s << endl;
    //cout << "r = " << r << endl;
}
void *Service_2(void *threadp)
{

    while(1)
    {   
        sem_wait(&semS2);

        //changes by tanmay
        clock_gettime(CLOCK_REALTIME, &start_s2);
        process_image(min_area, max_area); 
        clock_gettime(CLOCK_REALTIME, &end_s2);
        
        sem_post(&semS3);
    }

    pthread_exit((void *)0);
}

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


void *Service_4(void *threadp)
{
    char c;
    while(1)
    {   
        sem_wait(&semS4);

        clock_gettime(CLOCK_REALTIME, &start_s4);
        //~ UART_Transmit('a');
        //~ c = UART_Receive();
        //~ printf("%c\n", c);
        test();
        clock_gettime(CLOCK_REALTIME, &end_s4);
        clock_gettime(CLOCK_REALTIME, &end_total);
        
        sem_post(&semS5);
    }

    pthread_exit((void *)0);
}


void *Service_5(void *threadp)
{
    static uint32_t total_time_msec;
    char c;
    
    uint32_t s1_msec=0, s2_msec=0, s3_msec=0, s4_msec=0;
    
    while(1)
    {   
        sem_wait(&semS5);
        
        ctr++;
        
        cout << " distance: " << distance_udm << "cm" << endl;
        
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
        
        cout << "Observed time :" << total_time_msec << " msec" << endl;
        cout << "Estimated deadline: " << computed_deadline*1000 << " msec" << endl;
        
        if (total_time_msec < (computed_deadline*1000))
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


static void configure_services(void)
{
    int i, rc;
    cpu_set_t threadcpu;

    //update thread workers
    // threadParams[0].worker = Sequencer;
    // threadParams[1].worker = Service_1;
    // threadParams[2].worker = Service_2;
    // threadParams[3].worker = Service_3;

    for(i=0; i < NUM_SERVICES; i++)
    {

        CPU_ZERO(&threadcpu);
        CPU_SET(3, &threadcpu);

        rc=pthread_attr_init(&rt_sched_attr[i]);
        rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
        rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
        //rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

        rt_param[i].sched_priority = get_priority((service_t)i); 
        pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

        threadParams[i].threadIdx = i;
        threadParams[i].worker = get_worker((service_t)i);

        rc=pthread_create(&threads[i],              // pointer to thread descriptor
                        &rt_sched_attr[i],          // use specific attributes
                        //(void *)0,                // default attributes
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
void semaphores_init()
{
    if (sem_init (&semS1, 0, 0)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }
    if (sem_init (&semS2, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
    if (sem_init (&semS3, 0, 0)) { printf ("Failed to initialize S3 semaphore\n"); exit (-1); }
    if (sem_init (&semS4, 0, 0)) { printf ("Failed to initialize S4 semaphore\n"); exit (-1); }
    if (sem_init (&semS5, 0, 0)) { printf ("Failed to initialize S5 semaphore\n"); exit (-1); }
       
}

//changes by tanmay------------------------
/*
 * @brief: initializes camera and starts capturing frames
 * 
 * */
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
//-----------------------------------------------------

void sighandler(int sig_no){

    cout<<"SIGTERM detected!"<<endl;
    
    gpioTerminate();
    
    cout << "Average execution times: " << endl;
    
    cout << "For S1: " << service_times[SERVICE_1].total_ci/ctr << endl;
    cout << "For S2: " << service_times[SERVICE_2].total_ci/ctr << endl;
    cout << "For S3: " << service_times[SERVICE_3].total_ci/ctr << endl;
    cout << "For S4: " << service_times[SERVICE_4].total_ci/ctr << endl;
    
        
    cout << "WCET: " << endl;
    
    cout << "For S1: " << service_times[SERVICE_1].wcet << endl;
    cout << "For S2: " << service_times[SERVICE_2].wcet << endl;
    cout << "For S3: " << service_times[SERVICE_3].wcet << endl;
    cout << "For S4: " << service_times[SERVICE_4].wcet << endl;
    
    exit(0);
}

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
    
    //cout << "Distance: " << distance_cm << endl;
    
    cout << "-----------------------------------------------------------------------" << endl;
    
    cout << "Assuming red light is detected " << distance_cm << " cm from Car's current position:" << endl;
    cout << "Current speed of car: " << CURRENT_SPEED_OF_CAR << " cm/sec" << endl;
    time_to_stop_sec = time_to_stop_in_sec (CURRENT_SPEED_OF_CAR, distance_cm);
    cout << "Total time required for car to come to a full Stop: " << time_to_stop_sec << " seconds" << endl;
    
    computed_deadline = compute_deadline_to_complete_tasks (distance_cm);
    cout << "Computed deadline for all tasks: " << computed_deadline << " seconds" << endl;
    cout << "-----------------------------------------------------------------------" << endl;
    
    int i;
    //~ //for (i=SCHEDULER; i<=SERVICE_2; i++)      
    //~ //    printf("%d ", i);

    //~ //printf("\n");

    camera_init(); //tanmay
    
    udm_init();
    
    //UART_Init();
    
    configure_service_scheduler();

    configure_services();
    
    for(i=0;i<NUM_SERVICES;i++)
    {
        pthread_join(threads[i], NULL);
    }
    //test();
    
    printf("\nTEST COMPLETE\n");

   return 0;
}
