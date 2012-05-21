#ifndef PHIDGET_COMPASS_HH
#define PHIDGET_COMPASS_HH

#include <string>

#include <phidget21.h>

class PhidgetCompass {
  public:
    PhidgetCompass();
    ~PhidgetCompass();

    void set_tweaks(double x0, double x1, double y0, double y1);
    void set_east(double north);

    void write_cfg(std::string fn);
    void read_cfg(std::string fn);

    double get_bearing();
    double heading() { return get_bearing(); }

    CPhidgetSpatialHandle compass;

    double bearing;
    double raw_x;
    double raw_y;
    double raw_z;
};

#endif
