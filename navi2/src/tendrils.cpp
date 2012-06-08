#include <cmath>
#include <list>
#include <limits>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "transformHelper.hpp"

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void pathCallback(const nav_msgs::Path::ConstPtr& msg);
geometry_msgs::Twist castTendrils();
bool castTendril(double linear, double angular, double& distance);
bool At(int x, int y);

typedef struct
{
    double duration, resolution;
    double maxSpeed, maxTurn, cornering;
    int speedSteps;
    int turnSteps;
}RosParams;

typedef struct
{
    double xs, ys, theta;
}Pos;

RosParams params;
Pos pos;
std::list<geometry_msgs::PoseStamped> path;
nav_msgs::OccupancyGrid map;

ros::Publisher debug;
visualization_msgs::MarkerArray markers;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Tendrils");
    ros::NodeHandle nh;
    
    ros::Subscriber mapSub = nh.subscribe<nav_msgs::OccupancyGrid>("inputMap", 1, mapCallback);
    ros::Subscriber goalSub = nh.subscribe<nav_msgs::Path>("inputPath", 1, pathCallback);
    
    ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>("output", 1);
    debug = nh.advertise<visualization_msgs::MarkerArray>("debug", 1);
    
    ros::param::param<double>("~duration", params.duration, 0.7);
    ros::param::param<double>("~resolution", params.resolution, 0.1);
    
    ros::param::param<double>("~maxSpeed", params.maxSpeed, 2.0);
    ros::param::param<double>("~maxTurn", params.maxTurn, 1.2);
    ros::param::param<double>("~cornering", params.cornering, 0.4);
    
    ros::param::param<int>("~speedSteps", params.speedSteps, 5);
    ros::param::param<int>("~turnSteps", params.turnSteps, 4);
    
    ros::Rate rate(10);
    
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
        
        if (map.data.size() > 0 && path.size() > 0)
        {
            //castTendrils();
            velPub.publish(castTendrils());
        }
    }
}

geometry_msgs::Twist castTendrils()
{
    getTransform("/map", "/base_link", ros::Time::now(), ros::Duration(1.0), pos.xs, pos.ys, pos.theta);
 
    while (path.size() >= 1 && 
           hypot(pos.xs - path.back().pose.position.x,
                 pos.ys - path.back().pose.position.y) < params.duration * params.maxSpeed)
    {
        path.pop_back();
    }
    
    double bestL, bestA;
    double bestDist = hypot(pos.xs - path.back().pose.position.x,
                            pos.ys - path.back().pose.position.y) - 0.4;
    double dist;
    
    int index = 0, chosenIndex;
    
    markers = visualization_msgs::MarkerArray();
    
    for (int speedStep = 0; speedStep <= params.speedSteps; ++speedStep)
    {
        for (int turnStep = -params.turnSteps; turnStep <= params.turnSteps; ++turnStep)
        {
            double speed = speedStep * params.maxSpeed / params.speedSteps;
            double turn = turnStep * params.maxTurn / params.turnSteps;
            turn *= 1.0 - params.cornering * (speed / params.maxSpeed);
            
            if (castTendril(speed, turn, dist))
            {
                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestL = speed;
                    bestA = turn;
                    
                    chosenIndex = index;
                }
            }
            
            index++;
        }
    }
    
    //DEBUG
    {
        visualization_msgs::Marker mark;
        
        mark.header.stamp = ros::Time::now();
        mark.header.frame_id = "/map";
        mark.action = visualization_msgs::Marker::ADD;
        mark.ns = "TENDRILS";
        mark.id = 0;
        mark.type = visualization_msgs::Marker::SPHERE;
        mark.lifetime = ros::Duration();
        
        mark.scale.x = 0.1;
        mark.scale.y = 0.1;
        mark.scale.z = 0.1;
        
        mark.pose.position.x = path.back().pose.position.x;
        mark.pose.position.y = path.back().pose.position.y;
        
        mark.pose.orientation.x = 0.0;
        mark.pose.orientation.y = 0.0;
        mark.pose.orientation.z = 0.0;
        mark.pose.orientation.w = 1.0;
        
        mark.color.r = 1.0;
        mark.color.g = 1.0;
        mark.color.a = 1.0;
        
        markers.markers.push_back(mark);
    }
    
    geometry_msgs::Twist twist;
    
    if (abs(bestL) < 0.01)
    {
        //No good movments - turn towards goalSub
        
        double angDiff = atan2(path.back().pose.position.y - pos.ys,
                               path.back().pose.position.x - pos.xs);
        
        angDiff -= pos.theta;
        
        if (angDiff >  M_PI) angDiff -= 2.0 * M_PI;
        if (angDiff < -M_PI) angDiff += 2.0 * M_PI;
        
        if (angDiff < 0.0)
        {
            twist.angular.z = -params.maxTurn;
        }
        else
        {
            twist.angular.z = params.maxTurn;
        }
    }
    else
    {
        //DEBUG
        for (unsigned int i = 0; i < markers.markers[chosenIndex].colors.size(); i++)
        {
            markers.markers[chosenIndex].points[i].z = 0.1;
            
            markers.markers[chosenIndex].colors[i].r = 1.0;
            markers.markers[chosenIndex].colors[i].g = 1.0;
            markers.markers[chosenIndex].colors[i].b = 0.0;
        }
        
        twist.linear.x = bestL;
        twist.angular.z = -bestA;
    }
    
    debug.publish(markers);
    
    return twist;
}

bool castTendril(double linear, double angular, double& distance)
{
    //DEBUG
    {
        visualization_msgs::Marker mark;
        
        mark.header.stamp = ros::Time::now();
        mark.header.frame_id = "/map";
        mark.action = visualization_msgs::Marker::ADD;
        mark.ns = "TENDRILS";
        mark.id = markers.markers.size() + 1;
        mark.type = visualization_msgs::Marker::LINE_STRIP;
        mark.lifetime = ros::Duration();
        
        mark.scale.x = 0.01;
        
        mark.pose.orientation.x = 0.0;
        mark.pose.orientation.y = 0.0;
        mark.pose.orientation.z = 0.0;
        mark.pose.orientation.w = 1.0;
        
        mark.color.a = 1.0;
        
        markers.markers.push_back(mark);
    }
    
    distance = std::numeric_limits<double>::max();
    
    if (fabs(angular) < 0.01)
    {
        for (double time = 0.0; time <= params.duration; time += params.resolution)
        {
            double xx = pos.xs + linear * time * cos(pos.theta);
            double yy = pos.ys + linear * time * sin(pos.theta);
            
            //DEBUG
            {
                markers.markers.back().points.push_back(geometry_msgs::Point());
                markers.markers.back().points.back().x = xx;
                markers.markers.back().points.back().y = yy;
                markers.markers.back().points.back().z = 0.01;
                
                markers.markers.back().colors.push_back(std_msgs::ColorRGBA());
                markers.markers.back().colors.back().r = 0.0;
                markers.markers.back().colors.back().g = linear / params.maxSpeed;
                markers.markers.back().colors.back().b = 1.0 - linear / params.maxSpeed;
                markers.markers.back().colors.back().a = 1.0;
            }
            
            distance = std::min(distance, hypot(xx - path.back().pose.position.x,
                                                yy - path.back().pose.position.y));
            
            if (!(At(xx / map.info.resolution, yy / map.info.resolution)))
            {
                //DEBUG
                {
                    markers.markers.back().colors.back().r = 1.0;
                    markers.markers.back().colors.back().g = 0.0;
                    markers.markers.back().colors.back().b = 0.0;
                    markers.markers.back().colors.back().a = 1.0;
                }
                return false;
            }
        }
    }
    else
    {
        for (double time = 0.0; time <= params.duration; time += params.resolution)
        {
            double radius = 1.0 * (linear / angular) * sin(angular * time);
            double xx = pos.xs + radius * (cos(angular * time) * cos(pos.theta) +
                                           sin(angular * time) * sin(pos.theta));
            double yy = pos.ys + radius * (-sin(angular * time) * cos(pos.theta) +
                                            cos(angular * time) * sin(pos.theta));
            distance = std::min(distance, hypot(xx - path.back().pose.position.x,
                                                yy - path.back().pose.position.y));
            
            {//DEBUG
                markers.markers.back().points.push_back(geometry_msgs::Point());
                markers.markers.back().points.back().x = xx;
                markers.markers.back().points.back().y = yy;
                markers.markers.back().points.back().z = 0.01;
                
                markers.markers.back().colors.push_back(std_msgs::ColorRGBA());
                markers.markers.back().colors.back().r = 0.0;
                markers.markers.back().colors.back().g = linear / params.maxSpeed;
                markers.markers.back().colors.back().b = 1.0 - linear / params.maxSpeed;
                markers.markers.back().colors.back().a = 1.0;
            }
            
            if (!(At(xx / map.info.resolution, yy / map.info.resolution)))
            {
                {//DEBUG
                    markers.markers.back().colors.back().r = 1.0;
                    markers.markers.back().colors.back().g = 0.0;
                    markers.markers.back().colors.back().b = 0.0;
                    markers.markers.back().colors.back().a = 1.0;
                }
                return false;
            }
        }
    }
    
    return true;
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

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map = (*msg);
}

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    std::list<geometry_msgs::PoseStamped> tmpPath;
    
    for (std::vector<geometry_msgs::PoseStamped>::const_iterator i = msg->poses.begin(); i != msg->poses.end(); i++)
    {
        tmpPath.push_back(*i);
    }
    
    path = tmpPath;
}