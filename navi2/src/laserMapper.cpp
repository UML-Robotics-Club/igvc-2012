#include <cmath>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Path.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#include "transformHelper.hpp"


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
bool inBounds(int xx, int yy);
void inflateObstacle(int xx, int yy);
void raytrace(int xs, int ys, double xinc, double yinc, double dist);
void rayclear(int xs, int ys, int xt, int yt);

typedef struct
{
    double laserRange;
    double inflationRadius;
}RosParams;

RosParams rosParams;
ros::Publisher *laserMapPub;
nav_msgs::OccupancyGrid* laserGrid;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "LaserMapper");
    ros::NodeHandle nh;
    
    initTfHelper();
    
    laserGrid =  new nav_msgs::OccupancyGrid();
    
    laserGrid->info.origin.position.x = 0;
    laserGrid->info.origin.position.y = 0;
    laserGrid->info.origin.position.z = 0;
    laserGrid->info.origin.orientation.w = 1;
    laserGrid->info.origin.orientation.x = 0;
    laserGrid->info.origin.orientation.y = 0;
    laserGrid->info.origin.orientation.z = 0;
    
    double tmpRes;
    int tmpWidth, tmpHeight;
    
    ros::param::param<double>("~mapResolution", tmpRes, 0.2);
    ros::param::param<int>("~mapWidth", tmpWidth, 500);
    ros::param::param<int>("~mapHeight", tmpHeight, 500);
    
    laserGrid->info.resolution = tmpRes;
    laserGrid->info.width = tmpWidth;
    laserGrid->info.height = tmpHeight;
    
    ros::param::param<double>("~laserRange", rosParams.laserRange, 8.0);
    ros::param::param<double>("~inflationRadius", rosParams.inflationRadius, 0.6);
    
    for (unsigned int i = 0; i < laserGrid->info.height * laserGrid->info.width; ++i)
    {
        laserGrid->data.push_back(-1);
    }
    
    ros::Subscriber laserSub = nh.subscribe("input", 1, laserCallback);
    
    ros::Publisher laserMapPub_ = nh.advertise<nav_msgs::OccupancyGrid>("output", 1);
    laserMapPub = &laserMapPub_;
    
    ros::spin();
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    double xx, yy, yaw;
    
    getTransform("/map", msg->header.frame_id, msg->header.stamp, ros::Duration(1.0), xx, yy, yaw);
    
    double phi = msg->angle_min;
    unsigned int xr = xx / laserGrid->info.resolution;
    unsigned int yr = yy / laserGrid->info.resolution;
    for (unsigned int ii = 0; ii < msg->ranges.size(); ++ii, phi += msg->angle_increment)
    {
        double mapDist = msg->ranges[ii] / laserGrid->info.resolution;
        int xh = xr + cos(phi + yaw) * mapDist;
        int yh = yr + sin(phi + yaw) * mapDist;
        
        if (inBounds(xh, yh) && msg->ranges[ii] < rosParams.laserRange && msg->ranges[ii] > msg->range_min)
        {
            inflateObstacle(xh, yh);
        }
        
        if (msg->ranges[ii] - rosParams.inflationRadius > 0.0 && msg->ranges[ii] > msg->range_min)
        {
            raytrace(xr, yr, cos(phi + yaw), sin(phi + yaw), mapDist - rosParams.inflationRadius / laserGrid->info.resolution);
        }
    }
    
    double xb, yb;
    
    getTransform("/map", "base_link", msg->header.stamp, ros::Duration(1.0), xb, yb, yaw);
    rayclear(xb / laserGrid->info.resolution,
             yb / laserGrid->info.resolution, xr, yr);
    
    //laserGrid->data[xb / laserGrid->info.resolution + laserGrid->info.width * yb / laserGrid->info.resolution] = 0;
    
    laserGrid->header.stamp = msg->header.stamp;
    laserMapPub->publish(*laserGrid);
}

bool inBounds(int xx, int yy)
{
    return (xx >= 0) && (xx < laserGrid->info.width) && (yy >= 0) && (yy < laserGrid->info.height);
}

void inflateObstacle(int xx, int yy)
{
    int mapInflation = rosParams.inflationRadius / laserGrid->info.resolution;
    
    for (int xo = -mapInflation; xo <= mapInflation; ++xo)
    {
        for (int yo = -mapInflation; yo <= mapInflation; ++yo)
        {
            if (hypot(xo, yo) <= (rosParams.inflationRadius / laserGrid->info.resolution) && inBounds(xx + xo, yy + yo))
            {
                if (laserGrid->data[(xx + xo) + laserGrid->info.width * (yy + yo)] >= 98)
                {
                    laserGrid->data[(xx + xo) + laserGrid->info.width * (yy + yo)] = 100;
                }
                else
                {
                    laserGrid->data[(xx + xo) + laserGrid->info.width * (yy + yo)] += 2;
                }
            }
        }
    }
}

void raytrace(int xs, int ys, double xinc, double yinc, double dist)
{
    double xx = xs;
    double yy = ys;
    double curDist = 0;
    
    while (inBounds(xx, yy) && curDist < dist)
    {
        if (laserGrid->data[(int)xx + laserGrid->info.width * (int)yy] <= 0)
        {
            laserGrid->data[(int)xx + laserGrid->info.width * (int)yy] = 0;
        }
        else
        {
            laserGrid->data[(int)xx + laserGrid->info.width * (int)yy]--;
        }
        
        xx += xinc;
        yy += yinc;
        curDist += 1;
    }
}

void rayclear(int xs, int ys, int xt, int yt)
{
    double xx = xs;
    double yy = ys;
    
    double ang = atan2(yt - ys, xt - xs);
    double xinc = cos(ang) / 2.0;
    double yinc = sin(ang) / 2.0;
    
    double dist = hypot(xt - xs, yt - ys);
    double curDist = 0;
    
    while (inBounds(xx, yy) && curDist <= dist)
    {
        laserGrid->data[(int)xx + laserGrid->info.width * (int)yy] = 0;
        
        xx += xinc;
        yy += yinc;
        curDist += 1;
    }
}
