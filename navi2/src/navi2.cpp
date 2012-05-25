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
void laserMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

tf::TransformListener* tfListener;
nav_msgs::OccupancyGrid* laserGrid;
std::vector<nav_msgs::OccupancyGrid> maps;
Pathfinder* pather;
bool gotLaserMap = false;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Navi2");
    ros::NodeHandle nh;
    
    ros::Subscriber laserMapSub = nh.subscribe<nav_msgs::OccupancyGrid>("laserMap", 1, laserMapCallback);
    maps.push_back(nav_msgs::OccupancyGrid());
    
    ros::Subscriber goalSub = nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, goalCallback);
    
    ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("path", 1);
    
    tfListener = new tf::TransformListener();
    
    pather = new Pathfinder(nh);
    pather->SetTarget(0.0, 0.0);
    
    while (true)
    {
        ros::spinOnce();
        
        if (gotLaserMap)
        {
            pathPub.publish(pather->MakePath(maps));
        }
    }
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pather->SetTarget(msg->pose.position.x, msg->pose.position.y);
}

void laserMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    gotLaserMap = true;
    maps[0] = (*msg);
}
