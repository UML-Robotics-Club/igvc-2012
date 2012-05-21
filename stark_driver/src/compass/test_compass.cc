
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <sstream>

#include <stdlib.h>
#include <math.h>

#include "PhidgetCompass.hh"
#include "fsleep.hh"

//const double tweak = -1.15;

double
fmod(double xx, double yy)
{
    int    nn = (int)(xx / yy);
    double vv = xx - yy * nn;
    if (vv >= 0)
        return vv;
    else
        return vv + yy;
}

double
norm_angle(double aa)
{
    double bb = fmod(aa, 2*M_PI);
    if (bb < M_PI)
        return bb;
    else
        return bb - 2*M_PI;
}

void
run_bearing()
{
    std::ostringstream tmp;
    tmp << getenv("HOME");
    tmp << "/.magnet.tweaks";

    PhidgetCompass *compass = new PhidgetCompass;
    compass->read_cfg(tmp.str());
    
    while (1) {
        double ang = norm_angle(compass->heading());
        cout << ang << endl;
        fsleep(0.05);
    }

    delete compass;
}

int
main(int argc, char* argv[])
{
    run_bearing();
    return 0;
}
