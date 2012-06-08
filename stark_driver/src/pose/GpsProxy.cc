#include <iostream>

using std::cout;
using std::cerr;
using std::endl;

#include "GpsProxy.hh"
#include "CalcUTM.hh"

GpsProxy::GpsProxy()
{
    gps = new gpsmm;
    gps->open("localhost", DEFAULT_GPSD_PORT);
    gps->stream(WATCH_ENABLE|WATCH_JSON);
    fout_gps.open("raw_gps.txt");

}

GpsProxy::~GpsProxy()
{
    gps->close();
    delete gps;
    fout_gps.close();
}

void
GpsProxy::update()
{
    struct gps_data_t* msg;
    msg = gps->poll();
    double y = utm_north(msg->fix.latitude, msg->fix.longitude);
    double x = utm_east(msg->fix.latitude, msg->fix.longitude);
    std::string path(msg->dev.path);
    fout_gps << path << " " << x << " " << y << endl;

#if 0
    cout << "Polled dev: " << path
         << "; status =  " << msg->status 
         << "; sats = " << msg->satellites_used <<endl;
#endif

    if (msg->status && path != "") {
        if (data.find(path) == data.end()) {
            cout << "New device: " << path << endl;
        }

        data[path] = *msg;
    }
    else {
        cout << "No lock for device: " << path << endl;
    }
}

gps_pos_t
GpsProxy::position()
{
    gps_pos_t pos;

    int total_sats = 0;
    double lat = 0.0;
    double lon = 0.0;

    std::map<std::string,struct gps_data_t>::iterator it;
    for (it = data.begin(); it != data.end(); ++it) {
        struct gps_data_t dd = it->second;
        int sats = dd.satellites_used;

        if (sats > 0) {
            total_sats += sats;
            lat += sats * dd.fix.latitude;
            lon += sats * dd.fix.longitude;
        }
    }

    lat /= total_sats;
    lon /= total_sats;

    pos.utm_n = utm_north(lat, lon);
    pos.utm_e = utm_east(lat, lon);
    pos.lat   = lat;
    pos.lon   = lon;

    return pos;
}

