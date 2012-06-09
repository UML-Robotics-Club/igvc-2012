#include <cmath>
#include <cstdio>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"

#include "transformHelper.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Recorder");
    ros::NodeHandle nh;
    
    initTfHelper();
    
    ros::Duration(1.0).sleep();

    double xx, yy, yaw;
    getTransform("/base_link", "/gps", ros::Time::now(), ros::Duration(1.0), xx, yy, yaw);
    
    printf("%lf %lf\r\n", xx, yy);
}