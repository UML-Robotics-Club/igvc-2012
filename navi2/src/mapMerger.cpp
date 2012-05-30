#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#include <string>
#include <vector>
#include <algorithm>

#include <boost/bind.hpp>

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg, int index);

std::vector<nav_msgs::OccupancyGrid> maps;
std::vector<char> flags;

int main(int argc, char* argv[])
{
    std::vector<ros::Subscriber> subs;

    ros::init(argc, argv, "Navi2");
    ros::NodeHandle nh;
    
    XmlRpc::XmlRpcValue topicList;
    ros::param::get("~inputTopics", topicList);
    
    ROS_ASSERT(topicList.size() > 0);
    ROS_ASSERT(topicList.getType() == XmlRpc::XmlRpcValue::TypeArray);
    
    
    for (int i = 0; i < topicList.size(); ++i) 
    {
        ROS_ASSERT(topicList[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string topic = static_cast<std::string>(topicList[i]);
        
        maps.push_back(nav_msgs::OccupancyGrid());
        flags.push_back(0);
        
        subs.push_back(nh.subscribe<nav_msgs::OccupancyGrid>(topic, 1, boost::bind(mapCallback, _1, i)));
    }
    
    ros::Publisher mapPub = nh.advertise<nav_msgs::OccupancyGrid>("output", 1);
    ros::Rate rate(20);
    
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
        
        //Everyone ready?
        if ((unsigned int)std::count(flags.begin(), flags.end(), 1) == flags.size())
        {
            //merge
            nav_msgs::OccupancyGrid map;
            
            map.info = maps.front().info;
            
            for (unsigned int i = 0; i < map.info.height * map.info.width; ++i)
            {
                map.data.push_back(-1);
                
                for (std::vector<nav_msgs::OccupancyGrid>::iterator j = maps.begin(); j != maps.end(); ++j)
                {
                    map.data[i] = std::max(map.data[i], j->data[i]);
                }
            }
            
            mapPub.publish(map);
            
            //Make not ready
            std::fill(flags.begin(), flags.end(), 0);
        }
    }
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg, int index)
{
    flags[index] = 1;
    maps[index] = (*msg);
}