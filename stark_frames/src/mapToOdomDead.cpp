#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include <cmath>
#include <iostream>

tf::TransformListener* tfListener;
tf::TransformBroadcaster* tfBroadcaster;
tf::Transform latestTransform;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "mapToOdom");
    ros::NodeHandle nh;
    
    ros::Subscriber poseSub = nh.subscribe("input", 0, poseCallback);
    
    tfListener = new tf::TransformListener();
    tfBroadcaster = new tf::TransformBroadcaster();
    
    ros::Rate rate(10);
    
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        
        tfBroadcaster->sendTransform(tf::StampedTransform(latestTransform, ros::Time::now(), "/map", "/odom"));
    }
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tf::StampedTransform odomToBase, gpsToMap;
    
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
        tfListener->waitForTransform("/odom", "/base_link", msg->header.stamp, ros::Duration(1.0));
        tfListener->lookupTransform("/odom", "/base_link", msg->header.stamp, odomToBase);
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
    pos.setX(100);
    pos.setY(100);
    
    tf::Quaternion quat(0,0,0,1);
    
    //quat = quat * odomToBase.getRotation().inverse();
    //pos = pos - odomToBase.getOrigin();
    
    latestTransform = tf::Transform(quat, pos);
}