#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include <proj_api.h>
#include <cmath>
#include <iostream>

tf::TransformListener* tfListener;
tf::TransformBroadcaster* tfBroadcaster;
tf::Quaternion lastQuat;
nav_msgs::Odometry lastOdom;

projPJ pjLatLon, pjUTM;
double baseX, baseY;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

int main(int argc, char* argv[])
{
    pjLatLon = pj_init_plus("+proj=latlong +datum=WGS84");
    pjUTM = pj_init_plus("+proj=utm +zone=19 +datum=WGS84");
    
    baseX = -71.323866 / 180 * M_PI;
    baseY = 42.656516  / 180 * M_PI;
    
    pj_transform(pjLatLon, pjUTM, 1, 1, &baseX, &baseY, NULL);
    
    ros::init(argc, argv, "gpsStageTransformer");
    ros::NodeHandle nh;
    
    ros::Subscriber gpsSub = nh.subscribe("/robot/gps", 0, gpsCallback);
    ros::Subscriber odomSub = nh.subscribe("/robot/base_pose_ground_truth", 0, odomCallback);
    
    tfListener = new tf::TransformListener();
    tfBroadcaster = new tf::TransformBroadcaster();

    ROS_INFO("BASE UTM: %lf %lf", baseX, baseY);
    
    ros::spin();
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double r, p ,y;
    
    lastQuat = tf::Quaternion(msg->pose.pose.orientation.x,
                              msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z,
                              msg->pose.pose.orientation.w);
    
    btMatrix3x3(lastQuat).getRPY(r, p, y);
    
    lastQuat.setRPY(r, p, y + M_PI / 2.0);
    
    lastOdom = *msg;
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    tf::StampedTransform odomToBase;
    tf::Vector3 pos;
    tf::Quaternion quat = lastQuat;
    double y = msg->latitude  / 180.0 * M_PI;
    double x = msg->longitude / 180.0 * M_PI;
    
    pj_transform(pjLatLon, pjUTM, 1, 1, &x, &y, NULL);
    
    pos.setX(x - baseX);
    pos.setY(y - baseY);
    
     try
    {
        tfListener->waitForTransform("/odom", "/base_footprint", msg->header.stamp, ros::Duration(1.0));
        tfListener->lookupTransform("/odom", "/base_footprint", msg->header.stamp, odomToBase);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    
    quat = quat * odomToBase.getRotation().inverse();
    pos = pos - odomToBase.getOrigin();
    
    tf::Transform mapToOdom(quat, pos);
    
    tfBroadcaster->sendTransform(tf::StampedTransform(mapToOdom, lastOdom.header.stamp, "/map", "/odom"));
}