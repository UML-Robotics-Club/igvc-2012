#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include <cmath>
#include <iostream>

tf::TransformListener* tfListener;
tf::TransformBroadcaster* tfBroadcaster;

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "mapToOdom");
    ros::NodeHandle nh;
    
    ros::Subscriber poseSub = nh.subscribe("input", 0, poseCallback);
    
    tfListener = new tf::TransformListener();
    tfBroadcaster = new tf::TransformBroadcaster();
    
    ros::spin();
}

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    tf::StampedTransform odomToBase, gpsToMap;
    
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
    
    try
    {
        tfListener->waitForTransform("/odom", "/base_link", now, ros::Duration(1.0));
        tfListener->lookupTransform("/odom", "/base_link", now, odomToBase);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    
    /*tf::Quaternion MBR;
    MBR.setEuler(msg->theta, 0, 0);
    
    tf::Transform MB(MBR, tf::Point(msg->x - gpsToMap.getOrigin().x(), msg->y - gpsToMap.getOrigin().y(), 0.0));
    
    tf::Transform OBRI(odomToBase.getRotation().inverse(), tf::Point(0, 0, 0));
    
    tf::Transform OBTI(tf::Quaternion(0, 0, 0, 1), 
                       odomToBase.getOrigin() * -1.0);
    
    tfBroadcaster->sendTransform(tf::StampedTransform(OBTI * OBRI * MB, now, "/map", "/odom"));*/
    
    tf::Vector3 pos;
    pos.setX(msg->x - gpsToMap.getOrigin().x());
    pos.setY(msg->y - gpsToMap.getOrigin().y());
    
    tf::Quaternion quat;
    quat.setRPY(0, 0, msg->theta);
    
    quat = quat * odomToBase.getRotation().inverse();
    pos = pos - odomToBase.getOrigin();
    
    tf::Transform mapToOdom(quat, pos);
    
    tfBroadcaster->sendTransform(tf::StampedTransform(mapToOdom, now, "/map", "/odom"));
}