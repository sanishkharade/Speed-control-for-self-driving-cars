/**
 * @file    :   mutex.c
 * @brief   :   This program demonstrates mutex locking mechanism for a shared resource
 *
 * @author  :   Sanish Kharade and Shuran Xu
 * 
 * @date    :   February 20, 2022
 * 
 * @link    :   Man pages for Linux functions
*/

#include <stdio.h>
#include <stdint.h>

#include <stdlib.h>     // for malloc
#include <errno.h>      // for error codes

#include <time.h>       // for timespec
#include <pthread.h>    // for pthread functions
#include <unistd.h>     // for sleep

#include <string.h>

#define POLICY SCHED_FIFO

#define NUM_THREADS 2

pthread_t threads[NUM_THREADS];


struct Data
{
    int a, b, c;
    struct timespec timestamp;
    pthread_mutex_t mutex;
};

struct Data data = {.a=0, .b=0, .c=0};
/**
 * @brief   :   Function to update the data
 *              
 * @param   :   arg - argument passed to thread
 *
 * @return  :   void
 *              
*/
void* update_data(void* arg)
{
    int ret;
    while(1)
    {
        ret = pthread_mutex_lock(&data.mutex);
        if(ret)
        {
            perror("pthread_mutex_lock:");
            continue;
            //pthread_exit(&ret);
        }
        else
        {
            printf("Update thread : Acquired the mutex\n");
        }
        ret = clock_gettime(CLOCK_REALTIME, &(data.timestamp));
        if(ret)
        {
            perror("clockgettime");
            pthread_mutex_unlock(&data.mutex);
            continue;
            //pthread_exit(&ret);
        }
        data.a += 1;
        data.b += 1;
        data.c += 1;
        printf("Updated the data and simulating a very long task now\n");
        printf("\n");

        // Wait here forever thus simulating a deadlock or a very long thread function
        // Can weither use a while(1) loop or just comment the mutex_unlock function
        // If this thread never unlocks the mutex, the 2nd thread will keep waiting 
        while(1);
        
        pthread_mutex_unlock(&data.mutex);
        
    }
}
/**
 * @brief   :   Function to print the data
 *              
 * @param   :   arg - argument passed to thread
 *
 * @return  :   void
 *              
*/
void* print_data(void* arg)
{
    struct timespec *wait;
    int ret = 0;
    wait=(struct timespec *)(malloc(sizeof(struct timespec)));
    while(1)
    {
        // Get current time and wait for 10 seconds beyond that
        ret = clock_gettime(CLOCK_REALTIME, wait);
        if(ret)
        {
            perror("clockgettime");
            continue;
            //pthread_exit(&ret);
        }

        wait->tv_sec += 10;
        printf("Print thread: Waiting for mutex for 10 seconds\n");
        
        ret = pthread_mutex_timedlock(&data.mutex,wait);
        if(ret == 0)
        {
            // Acquired the mutex before timeout
            printf("Acquired the mutex before timeout!\n");
            printf("Time:\tsec = %ld, nsec = %ld\n", data.timestamp.tv_sec, data.timestamp.tv_nsec);
            printf("Data:\ta = %d, b = %d, c = %d\n", data.a, data.b, data.c);
            pthread_mutex_unlock(&data.mutex);
        }
        else if(ret == ETIMEDOUT)
        {
            printf("Timedout!\n");
            //add time in below print
            time_t T = time(NULL);
            struct tm tm = *localtime(&T);

            printf("No new data available at %02d:%02d:%02d\n", tm.tm_hour, tm.tm_min, tm.tm_sec);
            printf("\n");
        }
        else
        {
            perror("pthread_mutex_timedlock:");
            continue;
        }

    }
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
 * @brief   :   Application entry point
 *              
 * @param   :   argc    - no of arguments
 *              *argv   - pointer to argument array
 *
 * @return  :   void
 *              
*/
int main(int argc, char *argv[])
{

    struct timespec ts;
    int ret;

    ret = clock_gettime(CLOCK_REALTIME, &ts);
    if(ret)
    {
        perror("clockgettime");
    }
    else
    {
        printf("sec = %ld, nsec = %ld\n", ts.tv_sec, ts.tv_nsec);
    }

    configure_process_scheduler();
    
    static pthread_attr_t rt_sched_attr[NUM_THREADS];
    static struct sched_param rt_param[NUM_THREADS];

    pthread_attr_t main_sched_attr;

    // Set the parameter variables
    int rt_max_prio;

    struct sched_param main_param;

    // Initialize the attribute variables
    pthread_attr_init(&rt_sched_attr[0]);
    pthread_attr_init(&rt_sched_attr[1]);
    pthread_attr_init(&main_sched_attr);

    // Set the attributes for all threads
    pthread_attr_setinheritsched(&(rt_sched_attr[0]), PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&(rt_sched_attr[0]), POLICY);

    pthread_attr_setinheritsched(&rt_sched_attr[1], PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&rt_sched_attr[1], POLICY);

    pthread_attr_setinheritsched(&main_sched_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&main_sched_attr, POLICY);

    rt_max_prio = sched_get_priority_max(POLICY);

    // Set the priorities.
    main_param.sched_priority = rt_max_prio;
    (rt_param[0]).sched_priority = rt_max_prio-1;
    (rt_param[1]).sched_priority = rt_max_prio-1;

    int rc = sched_setscheduler(getpid(), POLICY, &main_param);
    if (rc == -1)
    {
        printf("ERROR: sched_setscheduler() : %s\n", strerror(errno)); 
        exit(EXIT_FAILURE);
    }

    pthread_attr_setschedparam(&rt_sched_attr[0], &(rt_param[0]));
    pthread_attr_setschedparam(&rt_sched_attr[1], &(rt_param[1]));
    pthread_attr_setschedparam(&main_sched_attr, &main_param);


    pthread_mutex_init(&data.mutex, NULL);
    //pthread_t threads[2];
    printf("Creating threads\n");
    rc = pthread_create(&(threads[0]), &rt_sched_attr[0], update_data, NULL);
    if (rc)
    {
        printf("ERROR: pthread_create1() : %s\n", strerror(rc));
        exit(EXIT_FAILURE);
    }
   
    rc = pthread_create(&(threads[1]), &rt_sched_attr[1], print_data, NULL);
    if (rc)
    {
        printf("ERROR: pthread_create2() : %s\n", strerror(rc));
        exit(EXIT_FAILURE);
    }   

    // Wait for application to complete
    for(int i=0;i<NUM_THREADS;i++)
        pthread_join(threads[i], NULL);


    return 0;
}

