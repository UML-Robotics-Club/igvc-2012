
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include "ros/ros.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/PoseStamped.h"
using geometry_msgs::PoseStamped;

#include "sensor_msgs/NavSatFix.h"
using sensor_msgs::NavSatFix;

#include "geometry_util/conversion.h"

#include "GpsProxy.hh"

double heading;
geometry_msgs::QuaternionStamped quat;

void
imu_data_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    double yaw, pitch, roll;
    geometry_util::GetEulerYPR(msg->quaternion, yaw, pitch, roll);
    heading = yaw;
    quat = *msg;
}

void
stark_pose_node()
{
    ros::NodeHandle node;
    ros::Publisher pos_topic = node.advertise<PoseStamped>("pose2d", 5);
    ros::Publisher gps_topic = node.advertise<NavSatFix>("gps", 5);
    ros::Rate loop_rate(10);

    ros::Subscriber imu = node.subscribe("orientation", 5, imu_data_callback);

    GpsProxy gps;

    while (ros::ok()) {
        
        gps.update();
        gps_pos_t gps_pos = gps.position();

        PoseStamped pose;
	pose.header.stamp = ros::Time::now();
        pose.pose.position.x = gps_pos.utm_e;
        pose.pose.position.y = gps_pos.utm_n;
	pose.pose.position.z = 0;
	
	tf::Quaternion tf_quat;
	tf_quat.setRPY(0,0,heading);

        pose.pose.orientation.x = tf_quat.getX();
	pose.pose.orientation.y = tf_quat.getY();
	pose.pose.orientation.z = tf_quat.getZ();
	pose.pose.orientation.w = tf_quat.getW();
        pos_topic.publish(pose);

        NavSatFix gps;
        gps.header.stamp = quat.header.stamp;
        gps.latitude  = gps_pos.lat;
        gps.longitude = gps_pos.lon;

        gps_topic.publish(gps);

        ros::spinOnce();
        loop_rate.sleep();
    }
}


int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "pose_drive");
    stark_pose_node();
    return 0;
}
