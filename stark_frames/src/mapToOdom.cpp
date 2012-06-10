#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include <cmath>
#include <iostream>

tf::TransformListener* tfListener;
tf::TransformBroadcaster* tfBroadcaster;
tf::Transform newestTrans;

void rotCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "mapToOdom");
    ros::NodeHandle nh;
    
    ros::Subscriber compSub = nh.subscribe("inputCompass", 0, rotCallback);
    ros::Subscriber poseSub = nh.subscribe("inputGPS", 0, poseCallback);
    
    tfListener = new tf::TransformListener();
    tfBroadcaster = new tf::TransformBroadcaster();
    
    ros::Rate rate(10);
    
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
        tfBroadcaster->sendTransform(tf::StampedTransform(newestTrans, ros::Time::now(), "/map", "/odom"));
    }
}

void rotCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    newestTrans.setRotation(tf::Quaternion(msg->quaternion.x,
                                           msg->quaternion.y,
                                           msg->quaternion.z,
                                           msg->quaternion.w));
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
    
    tf::Vector3 pos;
    pos.setX(msg->x - gpsToMap.getOrigin().x());
    pos.setY(msg->y - gpsToMap.getOrigin().y());
    
    tf::Quaternion quat = newestTrans.getRotation();
    
    tf::Transform mapToOdom(quat, pos);
    
    newestTrans = mapToOdom;
}