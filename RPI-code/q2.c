/**
 * @file    :   q2.c
 * @brief   :   This program demonstrates the use of pthread mutexes for locking resources
 *
 * @author  :   Shuran Xu and Sanish Kharade
 * 
 * @date    :   February 28, 2022
 * 
 * @link    :   https://man7.org/linux/man-pages/man3/pthread_mutex_lock.3p.html
 *              
*/
#define _GNU_SOURCE
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <sched.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>

#define NUM_THREADS         (2)
#define SAMPLE_PERIOD       (500000000UL) // delay for 500 msec, 1000 Hz
#define NANOSEC_PER_SEC     (1000000000)
#define NSEC_PER_MSEC       (1000000)
#define NSEC_PER_MICROSEC   (1000)
#define TRUE (1)
#define FALSE (0)


typedef void* (*worker_t)(void*);

typedef struct {
    struct timespec current_time;
    double X;
    double Y;
    double Z;
}navigational_state_t;

static cpu_set_t allcpuset;
static pthread_mutex_t mutex;
static navigational_state_t nav_state;
static cpu_set_t threadcpu;
static pthread_attr_t rt_sched_attr[NUM_THREADS];
static int rt_max_prio, rt_min_prio;
static struct sched_param rt_param[NUM_THREADS];

typedef struct
{
    int threadIdx;
    worker_t worker;
} threadParams_t;


// POSIX thread declarations and scheduling attributes
//
pthread_t threads[NUM_THREADS];
threadParams_t threadParams[NUM_THREADS];
/**
 * @brief   :   Function to generate a delay in nanoseconds
 *              
 * @param   :   period - no of nanoseconds of delay required
 *
 * @return  :   void
 *              
*/
static void delay_ns(unsigned long period)
{
    int secs = period / NANOSEC_PER_SEC;
    int nsecs = period - secs * NANOSEC_PER_SEC;
    struct timespec delay_time = {secs,nsecs}; // delay for 1 msec, 1000 Hz
    struct timespec remaining_time;
    double residual;
    int rc, delay_cnt=0;

    delay_cnt=0; residual=0.0;

    // This secondary while loop is for sleeping the specified period time
    // (1 msec). 
    do
    { 
        rc=nanosleep(&delay_time, &remaining_time);

        // Check if we were interrupted by a signal.
        // Increment looping delay.
        if(rc == EINTR)
        { 
            residual = remaining_time.tv_sec + ((double)remaining_time.tv_nsec / (double)NANOSEC_PER_SEC);

            if(residual > 0.0) printf("residual=%lf, sec=%d, nsec=%d\n", residual, \
            (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec);

            delay_cnt++;
        }
        else if(rc < 0)
        {
            perror("nanosleep failed\n");
            exit(-1);
        }
        
    } while((residual > 0.0) && (delay_cnt < 100));
}
/**
 * @brief   :   Function to print the scheduler properties
 *              
 * @param   :   none
 *
 * @return  :   void
 *              
*/
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
/**
 * @brief   :   Function to configure the scheduler properties
 *              
 * @param   :   none
 *
 * @return  :   void
 *              
*/
static void configure_process_scheduler()
{
    int rc, scope;
    struct sched_param main_param;
    pthread_attr_t main_attr;
    pid_t mainpid;

    // Record the main application's process ID
    mainpid=getpid();

    // Set the main process priority to the max value allowed by SCHED_FIFO.
    // Set scheduler for main process to SCHED_FIFO
    rc=sched_getparam(mainpid, &main_param);
    main_param.sched_priority=sched_get_priority_max(SCHED_FIFO);;
    rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if(rc < 0) perror("main_param");
    print_scheduler();

    // Get the contention scope for the threads of this application.
    pthread_attr_getscope(&main_attr, &scope);

    if(scope == PTHREAD_SCOPE_SYSTEM)
        printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
        printf("PTHREAD SCOPE PROCESS\n");
    else
        printf("PTHREAD SCOPE UNKNOWN\n");
}
/**
 * @brief   :   Functiom for updating the data
 *              
 * @param   :   *threadp - pointer to the thread
 *
 * @return  :   void
 *              
*/
void *updateStateThread(void *threadp)
{
    pthread_t thread;
    // local (stack variable) used to demonstrate part 1 of Q1
    threadParams_t *threadParams = (threadParams_t *)threadp;
    thread = pthread_self();
    printf("\nThread idx=%d running\n", threadParams->threadIdx);

    while(TRUE)
    {   
        //lock the mutex
        pthread_mutex_lock(&mutex);

        //update the clock time
        clock_gettime(CLOCK_REALTIME, &nav_state.current_time);
        
        //update X,Y,Z with random value
        // nav_state.X = rand() % 50;
        // nav_state.Y = rand() % 20;
        // nav_state.Z = rand() % 10;
        
        printf("Thread idx=%d: Data updated to the following:\n", threadParams->threadIdx);
        // global variable used to demonstrate part 2 of Q1  
        printf("Time: %ld sec, %ld msec (%ld microsec, %ld nsec)\n",    
        nav_state.current_time.tv_sec, (nav_state.current_time.tv_nsec / NSEC_PER_MSEC),
	    (nav_state.current_time.tv_nsec / NSEC_PER_MICROSEC), nav_state.current_time.tv_nsec);
        
        // printf("X = %lf\n",nav_state.X);
        // printf("Y = %lf\n",nav_state.Y);
        // printf("Z = %lf\n",nav_state.Z);
        
        //new line
        printf("\n\n");
        
        //unlock the mutex
        pthread_mutex_unlock(&mutex);
        
        //sleep for SAMPLE_PERIOD
        delay_ns(SAMPLE_PERIOD);
    }
}

/**
 * @brief   :   Functiom for getting (printing) the data
 *              
 * @param   :   *threadp - pointer to the thread
 *
 * @return  :   void
 *              
*/
void *getStateThread(void *threadp)
{
    pthread_t thread;
    
    // local (stack variable) used to demonstrate part 1 of Q1
    threadParams_t *threadParams = (threadParams_t *)threadp;
    thread = pthread_self();
    printf("\nThread idx=%d running\n", threadParams->threadIdx);

    //sleep for SAMPLE_PERIOD
    delay_ns(SAMPLE_PERIOD);

    while(TRUE)
    {
        //lock the mutex
        pthread_mutex_lock(&mutex);
        
        //read all data from nav_state
        printf("Thread idx=%d: Latest navigational state data obtained:\n", threadParams->threadIdx);
        // global variable used to demonstrate part 2 of Q1  
        printf("Time: %ld sec, %ld msec (%ld microsec, %ld nsec)\n",    
        nav_state.current_time.tv_sec, (nav_state.current_time.tv_nsec / NSEC_PER_MSEC),
	    (nav_state.current_time.tv_nsec / NSEC_PER_MICROSEC), nav_state.current_time.tv_nsec);
        
        // printf("X = %lf\n",nav_state.X);
        // printf("Y = %lf\n",nav_state.Y);
        // printf("Z = %lf\n",nav_state.Z);
        
        //new line
        printf("\n\n");

        //unlock the mutex
        pthread_mutex_unlock(&mutex);
        //sleep for SAMPLE_PERIOD
        delay_ns(SAMPLE_PERIOD);
    }
}
/**
 * @brief   :   Function to configure the thread properties
 *              
 * @param   :   none
 *
 * @return  :   void
 *              
*/
static void configure_threads()
{
    int i, rc;

    //update thread IDs
    threadParams[0].threadIdx=0;
    threadParams[1].threadIdx=1;

    //update thread workers
    threadParams[0].worker = updateStateThread;
    threadParams[1].worker = getStateThread;

    // For every thread (3 in this case) 
    // Set the thread parameters: SCHED_FIFO and priority.
    // Ultimately, this priority does not matter because we will individually
    // set priorities further down in the code.
    //
    // It is important that the scheduler thread be given the highest 
    // priority (in this case, from entry 0 of rt_param).
    for(i=0; i < NUM_THREADS; i++)
    {
        CPU_ZERO(&threadcpu);
        CPU_SET(1, &threadcpu); // we will run threads on CPU 1

        rc=pthread_attr_init(&rt_sched_attr[i]);
        rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
        rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
        rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

        rt_param[i].sched_priority=rt_max_prio-2;
        pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

        threadParams[i].threadIdx=i;
    }
    printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));
}
/**
 * @brief   :   Function to create the threads
 *              
 * @param   :   none
 *
 * @return  :   void
 *              
*/
static void launch_threads()
{
    int i,rc;
    for(i=NUM_THREADS-1;i>=0;i--){
        rc=pthread_create(&threads[i],             // pointer to thread descriptor
                        &rt_sched_attr[i],         // use specific attributes
                        //(void *)0,               // default attributes
                        threadParams[i].worker,    // thread function entry point
                        (void *)&(threadParams[i]) // parameters to pass in
                        );
        if(rc < 0)
            printf("pthread_create for service %d\r\n",i);
        else
            printf("pthread_create successful for service %d\r\n",i);
    }
}
/**
 * @brief   :   Application entry point
 *              
 * @param   :   argc    - no of arguments
 *              *argv   - pointer to argument array
 *
 * @return  :   void
 *              
*/
int main (int argc, char *argv[])
{
   int rc;
   int i;
   int run;
   time_t t;
   
   /* Intializes random number generator */
   srand((unsigned) time(&t));

   // Initialize mutex
   if(pthread_mutex_init(&mutex, NULL) != 0){
        perror("\n mutex init failed\n");
        return 1;
    }

    // Clear the CPU set. 
    CPU_ZERO(&allcpuset);

    // Configure the process Scheduler to be FIFO. 
    configure_process_scheduler();

    // Record the min and max priorities allowed with SCHED_FIFO
    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    // Print the min and max priorities for SCHED_FIFO
    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);

    // Configure threads
    configure_threads();
    
    // Create service threads which will block
    launch_threads();

    // Wait for application to complete
    for(i=0;i<NUM_THREADS;i++)
        pthread_join(threads[i], NULL);

}
