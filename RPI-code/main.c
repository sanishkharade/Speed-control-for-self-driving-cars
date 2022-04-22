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
    NUM_SERVICES
}service_t;

priority_t rt_max_prio, rt_min_prio;

pthread_t threads[NUM_THREADS];
threadParams_t threadParams[NUM_THREADS];

static struct sched_param rt_param[NUM_THREADS];
pthread_attr_t rt_sched_attr[NUM_THREADS];

sem_t semS1, semS2, semS3;
struct timeval start_time_val;

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
    struct timespec delay_time = {0,100000000}; // delay for 33.33 msec, 30 Hz For Q1
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
        printf("Sequencer thread prior to delay @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
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
        if((seqCnt % 10) == 0) sem_post(&semS1);

        // Service_2 = RT_MAX-2	@ 1 Hz
        if((seqCnt % 10) == 0) sem_post(&semS2);

        // Service_3 = RT_MAX-3	@ 0.5 Hz
        if((seqCnt % 10) == 0) sem_post(&semS3);

        // // Service_4 = RT_MAX-2	@ 1 Hz
        // if((seqCnt % 30) == 0) sem_post(&semS4);

        // // Service_5 = RT_MAX-3	@ 0.5 Hz
        // if((seqCnt % 60) == 0) sem_post(&semS5);

        // // Service_6 = RT_MAX-2	@ 1 Hz
        // if((seqCnt % 30) == 0) sem_post(&semS6);

        // // Service_7 = RT_MIN	0.1 Hz
        // if((seqCnt % 300) == 0) sem_post(&semS7);

        gettimeofday(&current_time_val, (struct timezone *)0);
        //printf("Sequencer release all sub-services @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    } while(1);

    sem_post(&semS1); sem_post(&semS2); sem_post(&semS3);
    // sem_post(&semS4); sem_post(&semS5); sem_post(&semS6);
    // sem_post(&semS7);
    // abortS1=TRUE; abortS2=TRUE; abortS3=TRUE;
    // abortS4=TRUE; abortS5=TRUE; abortS6=TRUE;
    // abortS7=TRUE;

    pthread_exit((void *)0);
}
void *Service_1(void *threadp)
{

    while(1)
    {
        sem_wait(&semS1);
        printf("Service 1\n");   
    }

    pthread_exit((void *)0);
}


void *Service_2(void *threadp)
{

    while(1)
    {   
        sem_wait(&semS2);
        printf("Service 2\n");
    }

    pthread_exit((void *)0);
}
void *Service_3(void *threadp)
{

    while(1)
    {   
        sem_wait(&semS3);
        printf("Service 3\n");
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

        rt_param[i].sched_priority = get_priority(i); 
        pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

        threadParams[i].threadIdx = i;
        threadParams[i].worker = get_worker(i);

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
       
}
void main()
{
    int i;
    for (i=SCHEDULER; i<=SERVICE_2; i++)      
        printf("%d ", i);

    printf("\n");


    configure_service_scheduler();

    configure_services();
    
    for(i=0;i<NUM_SERVICES;i++)
        pthread_join(threads[i], NULL);


   printf("\nTEST COMPLETE\n");

}