#include <cmath>
#include <cstdio>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"

#include "transformHelper.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "QualWps");
    ros::NodeHandle nh;
    
    initTfHelper();
    
    ros::Publisher goalPub = nh.advertise<geometry_msgs::PoseStamped>("output", 1);
    
    geometry_msgs::PoseStamped g;
    
    FILE* wpFile = fopen(argv[1], "r");
    
    ros::Duration(1.0).sleep();
    ros::Rate rate(10);
    
    while (fscanf(wpFile, "%lf %lf", &g.pose.position.x, &g.pose.position.y) > 0)
    {
        printf("GOING TO WPS: %lf %lf\r\n", g.pose.position.x, g.pose.position.y);
    
        g.header.frame_id = "/gps";
        g.header.stamp = ros::Time::now();
    
        g.pose.orientation.w = 1.0;
        
        goalPub.publish(g);
        
        double dist, xx, yy, yaw;
        
        do
        {
            rate.sleep();
            ros::spinOnce();
            
            getTransform("/gps", "/base_link", ros::Time::now(), ros::Duration(1.0), xx, yy, yaw);
        } while (hypot(xx - g.pose.position.x, yy - g.pose.position.y) > 0.8);
    }
    
    fclose(wpFile);
}