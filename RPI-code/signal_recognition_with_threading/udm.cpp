#include <pigpio.h>
#include <iostream>
#include <time.h>
#include "udm.h"
#include <signal.h>

#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#define TRIG 	23
#define ECHO 	24
#define speed 	17150


using namespace std;


struct timeval tv;

double get_instant()
{
	gettimeofday(&tv, NULL);

    return (double)tv.tv_sec + (double)tv.tv_usec * 0.000001;
}


void init_udm()
{
    gpioSetMode(TRIG, PI_OUTPUT);
    gpioSetMode(ECHO, PI_INPUT);
}

bool delay(int value, int limit = 1000000)
{
    for(int i = 0; gpioRead(ECHO) == value; ++i)
    {
        if(i >= limit)
            return false;
    }

    return true;
}

double get_distance()
{
    gpioWrite(TRIG, 0);
    usleep(100000);
    gpioWrite(TRIG, 1);
    usleep(10);
    gpioWrite(TRIG, 0);

    if(delay(0))
    {
        double start_pulse = get_instant();

        if(delay(1))
        {
            double end_pulse = get_instant();

            double time = end_pulse - start_pulse;
            double distance = time * speed;

            return distance;
        }
    }

    return 0.0 / 0.0;
}

/*
void sighandler(int sig_no){

    cout<<"SIGTERM detected!"<<endl;
    
    gpioTerminate();
    
    exit(sig_no);
}


int main () 
{
    
	//signal(SIGINT, sighandler);
	
	if(gpioInitialise() < 0){
		cout << "pigpio initialisation failed" << endl;
		signal(SIGINT, sighandler); 
	}
	else
	{
		signal(SIGINT, sighandler);
		cout << "pigpio initialisation ok" << endl;

        	init_udm();

	        for(int i = 0; true; ++i)
		{
            		double distance = get_distance();
		
	        	cout << i << " distance: " << distance << "cm" << endl;
		}
	}

	gpioTerminate();

	return 0; 
}*/
