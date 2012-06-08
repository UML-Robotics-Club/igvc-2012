#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "transformHelper.hpp"

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
bool At(int x, int y);

typedef struct
{
    double laserRange;
    double inflationRadius;
}RosParams;

RosParams rosParams;
ros::Publisher laserPub;
nav_msgs::OccupancyGrid map;
double raytrace(double xs, double ys, double theta);



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "WOAHMerger");
    ros::NodeHandle nh;
    
    ros::param::param<double>("~laserRange", rosParams.laserRange, 8.0);
    ros::param::param<double>("~inflationRadius", rosParams.inflationRadius, 0.6);
    
    ros::Subscriber laserSub = nh.subscribe("inputLaser", 1, laserCallback);
    ros::Subscriber mapSub = nh.subscribe("inputMap", 1, mapCallback);
    laserPub = nh.advertise<sensor_msgs::LaserScan>("output", 1);
    
    ros::spin();
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map = *msg;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::LaserScan newScan = *msg;
    double xx, yy, yaw;
    
    getTransform("/map", msg->header.frame_id, msg->header.stamp, ros::Duration(1.0), xx, yy, yaw);
    
    for (unsigned int i = 0; i < msg->ranges.size(); i++)
    {
        double theta = yaw + msg->angle_min + (int)i * msg->angle_increment;
        newScan.ranges[i] = std::min((double)msg->ranges[i], raytrace(xx, yy, theta));
    }
    
    newScan.range_max = rosParams.laserRange;
    
    laserPub.publish(newScan);
}

bool At(int x, int y)
{
    if ((x >= 0) && (x < (int)(map.info.width)) && 
        (y >= 0) && (y < (int)(map.info.height)))
    {
        return map.data[x + y * map.info.width] < 40;
    }
    
    return false;
}

double raytrace(double xs, double ys, double theta)
{
    double xInc = cos(theta);
    double yInc = sin(theta);
    double dist;
    
    for (dist = 0.0; dist < rosParams.laserRange; dist += 0.5 * map.info.resolution)
    {
        int xx = (xInc * dist + xs) /  map.info.resolution;
        int yy = (yInc * dist + ys) /  map.info.resolution;
        
        if (!At(xx, yy))
        {
            return dist;
        }
    }
    
    return rosParams.laserRange + 0.1;
}
    