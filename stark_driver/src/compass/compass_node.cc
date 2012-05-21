
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <sstream>

#include <stdlib.h>
#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"

#include "LinearMath/btQuaternion.h"

#include "PhidgetCompass.hh"
#include "fsleep.hh"

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
run_compass_node()
{
    ros::NodeHandle node;
    ros::Publisher pub = 
        node.advertise<geometry_msgs::PoseStamped>("imu_pose", 2);
    ros::Rate loop_rate(20);

    std::ostringstream tmp;
    tmp << getenv("HOME");
    tmp << "/.magnet.tweaks";

    PhidgetCompass *compass = new PhidgetCompass;
    compass->read_cfg(tmp.str());
    
    while (ros::ok()) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "imu_link";
        
        msg.pose.position.x = 0.0;
        msg.pose.position.y = 0.0;
        msg.pose.position.z = 0.0; 

        double theta = norm_angle(compass->heading());
         
        btQuaternion quat;
        quat.setRPY(0.0, 0.0, theta);
        msg.pose.orientation.w = quat.w();
        msg.pose.orientation.x = quat.x();
        msg.pose.orientation.y = quat.y();
        msg.pose.orientation.z = quat.z();

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete compass;
}

int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "talker");
    run_compass_node();
    return 0;
}
