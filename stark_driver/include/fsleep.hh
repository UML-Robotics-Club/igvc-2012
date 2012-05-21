#ifndef FSLEEP_HH
#define FSLEEP_HH

#include <unistd.h>

inline void fsleep(double s) {
    useconds_t us = (useconds_t) (s * 1000000);
    usleep(us);
}

#endif
