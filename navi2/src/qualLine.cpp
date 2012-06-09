#include <cmath>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "QualLine");
    ros::NodeHandle nh;
    
    ros::Publisher goalPub = nh.advertise<geometry_msgs::PoseStamped>("output", 1);
    
    geometry_msgs::PoseStamped g;
    
    g.header.frame_id = "/base_link";
    g.header.stamp = ros::Time::now();
    
    g.pose.position.x = 25.0;
    g.pose.orientation.w = 1.0;
    
    ros::Duration(1.0).sleep();
    
    goalPub.publish(g);
    
    ros::spin();
}