#include <pigpio.h>
#include <iostream>
#include <time.h>

#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>


using namespace std;


struct timeval tv;

double getTime()
{
	gettimeofday(&tv, NULL);

    return (double)tv.tv_sec + (double)tv.tv_usec * 0.000001;
}


int coefficient = 17150;//12550.0; // nalezy dobrac go doswiadczalnie

int TRIG = 23;
int ECHO = 24;

void initializePins()
{
    gpioSetMode(TRIG, PI_OUTPUT);
    gpioSetMode(ECHO, PI_INPUT);
}

bool waitValue(int value, int limit = 1000000) // limit = 1000000 has been selected empirically - for faster CPUs and bigger distance use bigger values (It is like timeout)
{
    for(int i = 0; gpioRead(ECHO) == value; ++i)
    {
        if(i >= limit)
            return false;
    }

    return true;
}

double detectDistance()
{
    gpioWrite(TRIG, 0);
    usleep(100000); // or 500000 if It will stop working properly
    gpioWrite(TRIG, 1);
    usleep(10);
    gpioWrite(TRIG, 0);

    if(waitValue(0))
    {
        double pulseStart = getTime();

        if(waitValue(1))
        {
            double pulseEnd = getTime();

            double duration = pulseEnd - pulseStart;
            double distance = duration * coefficient;

            return distance;
        }
    }

    return 0.0 / 0.0;
}

int main () 
{
	if(gpioInitialise() < 0){
		cout << "pigpio initialisation failed" << endl; 
	}
	else
	{
		cout << "pigpio initialisation ok" << endl;

        	initializePins();

	        for(int i = 0; true; ++i)
		{
            		double distance = detectDistance();
		
	        	cout << i << " distance: " << distance << "cm" << endl;
		}
	}

	gpioTerminate();

	return 0; 
}
