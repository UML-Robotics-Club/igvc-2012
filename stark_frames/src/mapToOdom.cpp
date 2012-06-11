#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/QuaternionStamped.h"
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
    tf::StampedTransform gpsToMap;
    
    ros::Time now = ros::Time::now();
    
    try
    {
        tfListener->waitForTransform("/gps", "/map", now, ros::Duration(1.0));
        tfListener->lookupTransform("/gps", "/map", now, gpsToMap);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    
    tf::Vector3 pos;
    pos.setX(msg->pose.position.x - gpsToMap.getOrigin().x());
    pos.setY(msg->pose.position.y - gpsToMap.getOrigin().y());

    tf::Quaternion quat(msg->pose.orientation.x,
                        msg->pose.orientation.y,
                        msg->pose.orientation.z,
                        msg->pose.orientation.w);
    
    tf::Transform mapToOdom(quat, pos);
    
    tfBroadcaster->sendTransform(tf::StampedTransform(mapToOdom, msg->header.stamp, "/map", "/odom"));
}
