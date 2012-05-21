
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <fstream>

#include <stdlib.h>
#include <math.h>

#include "PhidgetCompass.hh"

PhidgetCompass* instance = 0;

// Offset variables
double xx0 = -0.7637;
double xx1 = -0.22884;
double yy0 = -0.1853;
double yy1 =  0.20715;
double aa0 =  0.0;

int got_compass_attach(CPhidgetHandle spatial, void *userptr)
{
    cerr << "Compass connected" << endl;
    return 0;
}

int got_compass_detatch(CPhidgetHandle spatial, void *userptr)
{
    cerr << "Compass disconnected" << endl;
    return 0;
}

int got_compass_error(CPhidgetHandle spatial, void *userptr, 
    int ErrorCode, const char *message)
{
    cerr << "Compass error: " << ErrorCode << " " << message << endl;
    return 0;
}

double normalize(double xx, double lo, double hi)
{
    double range = hi - lo;
    double hrng  = range / 2.0;
    double mid   = lo + hrng;
    double adj   = xx - mid;
    return adj / hrng;
}

int got_compass_data(CPhidgetSpatialHandle spatial, void *userptr, 
    CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
    for (int ii = 0; ii < count; ++ii) {
        double xx = data[ii]->magneticField[0];
        double yy = data[ii]->magneticField[1];
        double zz = data[ii]->magneticField[2];

        if (xx > 1e200 || yy > 1e200 || zz > 1e200)
            continue;

        instance->raw_x = xx;
        instance->raw_y = yy;
        instance->raw_z = zz;

        double nx = normalize(xx, xx0, xx1);
        double ny = normalize(yy, yy0, yy1);

        double ang = atan2(ny, nx);
        instance->bearing = aa0 + ang;
   }
   return 0;
}

PhidgetCompass::PhidgetCompass()
{
    cerr << "Connecting to Phidget compass..." << endl;

    if (instance) {
        compass = instance->compass;
        return;
    }

    instance = this;
    bearing  = 0.0;

    CPhidgetSpatial_create(&compass);
    
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)compass, got_compass_attach, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)compass, got_compass_detatch, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)compass, got_compass_error, NULL);

    CPhidgetSpatial_set_OnSpatialData_Handler(compass, got_compass_data, NULL);
    CPhidgetSpatial_setDataRate(compass, 100);
    
    CPhidgetSpatial_resetCompassCorrectionParameters(compass);

#if 0
    CPhidgetSpatial_setCompassCorrectionParameters(compass,
0.679136, 0.499077, 0.064415, 0.000000, 2.575526, 2.699631, 2.637578, -0.000727, 0.000000, -0.000694, 0.000000, 0.000000, 0.000000
	);
#endif

    CPhidget_open((CPhidgetHandle)compass, -1);

    cerr << "Compass up and limping." << endl;
}

PhidgetCompass::~PhidgetCompass()
{
    if (this == instance) {
        CPhidget_close((CPhidgetHandle)compass);
        CPhidget_delete((CPhidgetHandle)compass);
    }
}

double 
PhidgetCompass::get_bearing() {
    return bearing;
}

void
PhidgetCompass::set_tweaks(double x0, double x1, double y0, double y1)
{
    xx0 = x0;
    xx1 = x1;
    yy0 = y0;
    yy1 = y1;
}

void
PhidgetCompass::set_east(double north)
{
    aa0 = north;
}

void
PhidgetCompass::read_cfg(std::string fn)
{
    std::ifstream cfg(fn.c_str());
    cfg >> xx0 >> xx1 >> yy0 >> yy1 >> aa0;
}

void
PhidgetCompass::write_cfg(std::string fn)
{
    std::ofstream cfg(fn.c_str());
    cfg << xx0 << " " << xx1 << " " << yy0 << " " << yy1 << " " << aa0 << endl;
}
