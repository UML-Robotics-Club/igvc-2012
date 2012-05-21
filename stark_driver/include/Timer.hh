#ifndef TIMER_HH
#define TIMER_HH

#include <cstdio>
#include <sys/time.h>

inline double time_nowf() {
    static const double M = 1000000.0;
    struct timeval tv;
    gettimeofday(&tv, 0);
    return double(tv.tv_sec) + (double(tv.tv_usec) / M);
}

class Timer {
  public:
    Timer(std::string n) : name(n) {
        start = time_nowf();
    }
    
    inline double time() {
        double now = time_nowf();
        return now - start;
    }
    
    inline void print() {
        double tt = time();
        printf("Timer %s: %.04f\n", name.c_str(), tt);
    }

  private:    
    double start;
    std::string name;
};

#endif
