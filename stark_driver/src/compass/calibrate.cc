
#include <stdio.h>
#include <sys/select.h>
#include <pthread.h>
#include <math.h>
#include <stdlib.h>
#include <sstream>

#include "PhidgetCompass.hh"
#include "fsleep.hh"
    
volatile double min_x = 100;
volatile double max_x = -100;
volatile double min_y = 100;
volatile double max_y = -100;
volatile double north = 0.0;

pthread_t sample_thread;
volatile int keep_sampling;

void*
sample_thread_main(void* ptr)
{
    PhidgetCompass* compass = (PhidgetCompass*)ptr;

    while(fabs(compass->raw_x) < 0.001)
        fsleep(0.001);

    while (keep_sampling) {
        min_x = compass->raw_x < min_x ? compass->raw_x : min_x;        
        max_x = compass->raw_x > max_x ? compass->raw_x : max_x;        
        min_y = compass->raw_y < min_y ? compass->raw_y : min_y;        
        max_y = compass->raw_y > max_y ? compass->raw_y : max_y;
        fsleep(0.05);

        printf("tweaks = %.04lf %.04lf %.04lf %.04lf\n", min_x, max_x, min_y, max_y);
    }
}


void
collect_extremes(PhidgetCompass& compass)
{
    keep_sampling = 1;
    pthread_create(&sample_thread, 0, sample_thread_main, &compass);
    fsleep(0.5);

    printf("Ready to calibrate compass.\n\n");
    printf("Please spin the robot in place for three rotations,\n");
    printf("then press <Enter>.\n");

    getchar();
    keep_sampling = 0;
    fsleep(0.1);

    printf("\nCompass tweaks: %.04lf, %.04lf, %.04lf, %.04lf\n\n",
        min_x, max_x, min_y, max_y);
    compass.set_tweaks(min_x, max_x, min_y, max_y);
}

void
find_north(PhidgetCompass& compass)
{
    printf("Now we need to find east. Point the robot that way and wonk enter.\n");
    getchar();

    printf("\nCollecting samples...\n");

    const int SAMPLES = 10;
    double sum = 0.0;

    compass.set_east(0.0);
    
    for (int i = 0; i < SAMPLES; ++i) {
        sum += compass.heading();
        fsleep(0.25);
    }

    north = sum / SAMPLES;
    
    compass.set_east(-north);
}

void
run_calibrate()
{
    std::ostringstream tmp;
    tmp << getenv("HOME");
    tmp << "/.magnet.tweaks";

    PhidgetCompass compass;

    collect_extremes(compass);
    find_north(compass);
    
    compass.write_cfg(tmp.str());

    printf("\nCompass is calibrated, all done.\n\n");
}

int
main(int argc, char* argv[])
{
    run_calibrate();
    return 0;
}

