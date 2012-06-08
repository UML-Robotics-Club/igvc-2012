#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include <cmath>
#include <iostream>

tf::TransformListener* tfListener;
tf::TransformBroadcaster* tfBroadcaster;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "mapToOdom");
    ros::NodeHandle nh;
    
    ros::Subscriber poseSub = nh.subscribe("input", 0, poseCallback);
    
    tfListener = new tf::TransformListener();
    tfBroadcaster = new tf::TransformBroadcaster();
    
    ros::spin();
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tf::StampedTransform odomToBase, gpsToMap;
    tf::Vector3 pos;
    tf::Quaternion quat = tf::Quaternion(msg->pose.orientation.x,
                                         msg->pose.orientation.y,
                                         msg->pose.orientation.z,
                                         msg->pose.orientation.w);
    
    try
    {
        tfListener->waitForTransform("/gps", "/map", msg->header.stamp, ros::Duration(1.0));
        tfListener->lookupTransform("/gps", "/map", msg->header.stamp, gpsToMap);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    
    try
    {
        tfListener->waitForTransform("/odom", "/base_footprint", msg->header.stamp, ros::Duration(1.0));
        tfListener->lookupTransform("/odom", "/base_footprint", msg->header.stamp, odomToBase);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    
    pos.setX(msg->pose.position.x - gpsToMap.getOrigin().x());
    pos.setY(msg->pose.position.y - gpsToMap.getOrigin().y());
    
    quat = quat * odomToBase.getRotation().inverse();
    pos = pos - odomToBase.getOrigin();
    
    tf::Transform mapToOdom(quat, pos);
    
    tfBroadcaster->sendTransform(tf::StampedTransform(mapToOdom, msg->header.stamp, "/map", "/odom"));
}