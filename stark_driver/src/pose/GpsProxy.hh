#ifndef GPS_PROXY_HH
#define GPS_PROXY_HH

#include <map>
#include <libgpsmm.h>

// GpsProxy connects to a gpsd handling multiple GPS
// devices and combines all the data into one position
// expressed in UTM coordinates.

typedef struct gps_pos_t {
    double utm_n;
    double utm_e;
    double lat;
    double lon;
} gps_pos_t;

class GpsProxy {
  public:
    GpsProxy();
    ~GpsProxy();

    gps_pos_t position();
    void update();

  private:
    std::map<std::string,struct gps_data_t> data;
    gpsmm* gps;
};

#endif
