#ifndef GPS_PROXY_HH
#define GPS_PROXY_HH

#include <fstream>
#include <map>
#include <libgpsmm.h>
#include "robot_ekf.hpp"
#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

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
    std::ofstream fout_gps; // file to optain raw gps x,y data
    std::ofstream fout_kf_gps;
    std::ofstream fout_encoder_pos;
    RobotEKF gps0_ekf;
    RobotEKF gps1_ekf;
    tf::TransformListener tf_listener;
    tf::StampedTransform gps0_last_tf, gps1_last_tf;
    geometry_msgs::PoseStamped gps0_prev_enc_msg, gps1_prev_enc_msg;
    double encoder_x_pos, encoder_y_pos; // for testing control data
};

#endif
