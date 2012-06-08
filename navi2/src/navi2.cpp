#include <cmath>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"

#include "pathfinder.hpp"

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

tf::TransformListener* tfListener;
nav_msgs::OccupancyGrid* laserGrid;
nav_msgs::OccupancyGrid map;
Pathfinder* pather;
bool gotMap = false;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Navi2");
    ros::NodeHandle nh;
    
    ros::Subscriber mapSub = nh.subscribe<nav_msgs::OccupancyGrid>("inputMap", 1, mapCallback);
    
    ros::Subscriber goalSub = nh.subscribe<geometry_msgs::PoseStamped>("inputGoal", 1, goalCallback);
    
    ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("output", 1);
    
    tfListener = new tf::TransformListener();
    
    pather = new Pathfinder(nh);
    
    while (ros::ok())
    {
        ros::spinOnce();
        
        if (gotMap)
        {
            pathPub.publish(pather->MakePath(map));
        }
    }
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double xx, yy, yaw;
    getTransform("/map", msg->header.frame_id, msg->header.stamp, ros::Duration(1.0), xx, yy, yaw);
    
    pather->SetTarget(xx + cos(yaw) * msg->pose.position.x + sin(yaw) * msg->pose.position.y, 
                      yy - sin(yaw) * msg->pose.position.x + cos(yaw) * msg->pose.position.y);
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    gotMap = true;
    map = (*msg);
}
