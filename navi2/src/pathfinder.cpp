#include "pathfinder.hpp"

SearchNode::SearchNode()
{
    m_parent = NULL;
}

SearchNode::SearchNode(int x, int y, double cost, SearchNode* parent)
{
    m_x = x;
    m_y = y;
    
    m_cost = cost;
    m_parent = parent;
}

int SearchNodeCmp::m_xt;
int SearchNodeCmp::m_yt;

bool SearchNodeCmp::operator()(const SearchNode* a, const SearchNode* b)
{
    return (hypot(a->m_x - m_xt, a->m_y - m_yt) + a->m_cost) > 
           (hypot(b->m_x - m_xt, b->m_y - m_yt) + b->m_cost);
}

void SearchNodeCmp::SetTarget(int xt, int yt)
{
    m_xt = xt;
    m_yt = yt;
}

Pathfinder::Pathfinder(ros::NodeHandle& nh) : m_nh(nh)
{
    SetTarget(0, 0);
 
    m_poses = nh.advertise<geometry_msgs::PoseArray>("path_debug", 1);
    m_debug = nh.advertise<visualization_msgs::Marker>("path_debug2", 1);
    
    ros::param::param<double>("~pathfinderTolerance", m_tolerance, 0.4);
}

void Pathfinder::SetTarget(double xt, double yt)
{
    m_xt = xt;
    m_yt = yt;
}

nav_msgs::Path Pathfinder::MakePath(std::vector<nav_msgs::OccupancyGrid>& maps)
{
    SearchNodeCmp::SetTarget(m_xt / maps.front().info.resolution, m_yt / maps.front().info.resolution);
    std::priority_queue<SearchNode*, std::vector<SearchNode*>, SearchNodeCmp> queue;
    
    tf::StampedTransform transform;
    try
    {
        ros::Time stamp = ros::Time::now();
        
        m_tfListener.waitForTransform("/map", "/base_footprint", stamp, ros::Duration(1.0));
        m_tfListener.lookupTransform("/map", "base_footprint", stamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    
    m_searchmap = new SearchNode[maps.front().info.height * maps.front().info.width];
    for (unsigned int y = 0; y < maps.front().info.height; ++y)
    {
        for (unsigned int x = 0; x < maps.front().info.width; ++x)
        {
            m_searchmap[y * maps.front().info.width + x].m_x = x;
            m_searchmap[y * maps.front().info.width + x].m_y = y;
        }
    }
    
    int xx = transform.getOrigin().x() / maps.front().info.resolution;
    int yy = transform.getOrigin().y() / maps.front().info.resolution;
    
    /* Should get current vel */
    queue.push(&(m_searchmap[yy * maps.front().info.width + xx]));
    queue.top()->m_cost = 0;
    queue.top()->m_parent = HOME_NODE;
    
    ROS_INFO("START (FL): %lf, %lf", transform.getOrigin().x(), transform.getOrigin().y());
    ROS_INFO("TARGE (FL): %lf, %lf", m_xt, m_yt);
    
    ROS_INFO("NODE DETAILS: %d %d %lf", 
             m_searchmap[yy * maps.front().info.width + xx].m_x,
             m_searchmap[yy * maps.front().info.width + xx].m_y,
             m_searchmap[yy * maps.front().info.width + xx].m_cost);
    
    geometry_msgs::PoseArray markers;
    markers.header.frame_id = "/map";
    
    while (!queue.empty() && queue.size() < 500000)
    {
        SearchNode* nxt;
        SearchNode* cur = queue.top();
        queue.pop();
        
        
        {
            geometry_msgs::Pose marker;
            
            marker.position.x = cur->m_x * maps.front().info.resolution;
            marker.position.y = cur->m_y * maps.front().info.resolution;
            
            double yaw = 0;
            
            if (cur->m_parent > HOME_NODE)
            {
                yaw = atan2(cur->m_parent->m_y - cur->m_y,
                            cur->m_parent->m_x - cur->m_x);
            }
            
            tf::Quaternion quat;
            quat.setRPY(0.0, 0.0, yaw);
            
            marker.orientation.x = quat.getX();
            marker.orientation.y = quat.getY();
            marker.orientation.z = quat.getZ();
            marker.orientation.w = quat.getW();
            
            markers.poses.push_back(marker);
        }
        
        //ROS_INFO("NODE(%d): %lf, %lf", queue.size(), cur->m_x, cur->m_y);
        
        if (hypot(cur->m_x - m_xt / maps.front().info.resolution, 
                  cur->m_y - m_yt / maps.front().info.resolution) <= m_tolerance)
        {
            ROS_INFO("DONE");
            
            m_lastPath = nav_msgs::Path();
            m_lastPath.header.frame_id = "/map";
            
            while (cur != HOME_NODE)
            {
                ROS_INFO("AS:%d %d-%d", m_lastPath.poses.size(), cur->m_x, cur->m_y);
                geometry_msgs::PoseStamped pose;
                
                pose.header.frame_id = "/map";
                pose.pose.position.x = cur->m_x * maps.front().info.resolution;
                pose.pose.position.y = cur->m_y * maps.front().info.resolution;
                
                double yaw = 0;
                
                if (cur->m_parent > HOME_NODE)
                {
                    yaw = atan2(cur->m_parent->m_y - cur->m_y,
                                cur->m_parent->m_x - cur->m_x);
                }
                
                tf::Quaternion quat;
                quat.setRPY(0.0, 0.0, yaw);
                
                pose.pose.orientation.x = quat.getX();
                pose.pose.orientation.y = quat.getY();
                pose.pose.orientation.z = quat.getZ();
                pose.pose.orientation.w = quat.getW();
                
                m_lastPath.poses.push_back(pose);
                
                cur = cur->m_parent;
            }
            
            m_poses.publish(markers);
            
            delete[] m_searchmap;
            
            return GetLastPath();
        }
        else
        {
            //Expand Nodes
            
            int mx = cur->m_x;
            int my = cur->m_y;
            int nx, ny;
            
            for (int xmod = -1; xmod <= 1; xmod++)
            {
                for (int ymod = -1; ymod <= 1; ymod++)
                {
                    if (xmod != 0 || ymod != 0)
                    {
                        nx = mx + xmod;
                        ny = my + ymod;
                        
                        if (At(maps, nx, ny)) 
                        {
                            nxt = &(m_searchmap[ny * maps.front().info.width + nx]);
                            
                            if (nxt->m_parent == NULL || (nxt->m_parent != HOME_NODE && nxt->m_parent->m_cost > cur->m_cost))
                            {
                                
                                if (cur->m_parent > HOME_NODE && LOS(maps, cur->m_parent, nxt))
                                {
                                    nxt->m_cost = cur->m_parent->m_cost + hypot(nxt->m_x - cur->m_parent->m_x, 
                                                                                nxt->m_y - cur->m_parent->m_y);
                                    nxt->m_parent = cur->m_parent;
                                    
                                    if (cur->m_parent > HOME_NODE)
                                    {
                                        visualization_msgs::Marker mark;
                                        
                                        mark.header.stamp = ros::Time::now();
                                        mark.header.frame_id = "/map";
                                        mark.action = visualization_msgs::Marker::ADD;
                                        mark.ns = "P_LOS";
                                        mark.id = nx + ny * 1000;
                                        mark.type = visualization_msgs::Marker::LINE_STRIP;
                                        mark.lifetime = ros::Duration();
                                        
                                        mark.scale.x = 0.01;
                                        
                                        mark.pose.orientation.x = 0.0;
                                        mark.pose.orientation.y = 0.0;
                                        mark.pose.orientation.z = 0.0;
                                        mark.pose.orientation.w = 1.0;
                                        
                                        mark.color.r = 1.0;
                                        mark.color.g = 0.0;
                                        mark.color.b = 0.0;
                                        mark.color.a = 1.0;
                                        
                                        mark.points.push_back(geometry_msgs::Point());
                                        mark.points.back().x = cur->m_parent->m_x * maps.front().info.resolution;
                                        mark.points.back().y = cur->m_parent->m_y * maps.front().info.resolution;
                                        mark.points.back().z = cur->m_parent->m_cost / 50.0;
                                        
                                        mark.points.push_back(geometry_msgs::Point());
                                        mark.points.back().x = nxt->m_x * maps.front().info.resolution;
                                        mark.points.back().y = nxt->m_y * maps.front().info.resolution;
                                        mark.points.back().z = nxt->m_cost / 50.0;
                                        
                                        mark.colors.push_back(std_msgs::ColorRGBA());
                                        mark.colors.back().r = 0.0;
                                        mark.colors.back().g = 1.0;
                                        mark.colors.back().b = 0.0;
                                        mark.colors.back().a = 1.0;
                                        
                                        mark.colors.push_back(std_msgs::ColorRGBA());
                                        mark.colors.back().r = 0.0;
                                        mark.colors.back().g = 1.0;
                                        mark.colors.back().b = 0.0;
                                        mark.colors.back().a = 1.0;
                                        
                                        m_debug.publish(mark);
                                    }
                                    
                                }
                                else
                                {
                                    nxt->m_cost = cur->m_cost + hypot(nxt->m_x - cur->m_x, 
                                                                      nxt->m_y - cur->m_y);
                                    nxt->m_parent = cur;
                                    
                                    if (cur->m_parent > HOME_NODE)
                                    {
                                        visualization_msgs::Marker mark;
                                        
                                        mark.header.stamp = ros::Time::now();
                                        mark.header.frame_id = "/map";
                                        mark.action = visualization_msgs::Marker::ADD;
                                        mark.ns = "F_LOS";
                                        mark.id = nx + ny * 1000;
                                        mark.type = visualization_msgs::Marker::LINE_STRIP;
                                        mark.lifetime = ros::Duration();
                                        
                                        mark.scale.x = 0.01;
                                        
                                        mark.pose.orientation.x = 0.0;
                                        mark.pose.orientation.y = 0.0;
                                        mark.pose.orientation.z = 0.0;
                                        mark.pose.orientation.w = 1.0;
                                        
                                        mark.color.r = 1.0;
                                        mark.color.g = 0.0;
                                        mark.color.b = 0.0;
                                        mark.color.a = 1.0;
                                        
                                        mark.points.push_back(geometry_msgs::Point());
                                        mark.points.back().x = cur->m_parent->m_x * maps.front().info.resolution;
                                        mark.points.back().y = cur->m_parent->m_y * maps.front().info.resolution;
                                        mark.points.back().z = cur->m_parent->m_cost / 50.0;
                                        
                                        mark.points.push_back(geometry_msgs::Point());
                                        mark.points.back().x = nxt->m_x * maps.front().info.resolution;
                                        mark.points.back().y = nxt->m_y * maps.front().info.resolution;
                                        mark.points.back().z = nxt->m_cost / 50.0;
                                        
                                        mark.colors.push_back(std_msgs::ColorRGBA());
                                        mark.colors.back().r = 1.0;
                                        mark.colors.back().g = 0.0;
                                        mark.colors.back().b = 0.0;
                                        mark.colors.back().a = 1.0;
                                        
                                        mark.colors.push_back(std_msgs::ColorRGBA());
                                        mark.colors.back().r = 1.0;
                                        mark.colors.back().g = 0.0;
                                        mark.colors.back().b = 0.0;
                                        mark.colors.back().a = 1.0;
                                        
                                        m_debug.publish(mark);
                                    }
                                }
                                
                                queue.push(nxt);
                            }
                        }
                    }
                }
            }
        }
    }
    
    if (queue.size() >= 500000)
    {
        ROS_INFO("TERMINATED DUE TO # OF NODES");
    }
    else
    {
        ROS_INFO("TERMINATED DUE TO FAIL");
    }
    
    m_poses.publish(markers);
    
    delete[] m_searchmap;
    
    return GetLastPath();
}

nav_msgs::Path Pathfinder::GetLastPath()
{
    return m_lastPath;
}

bool Pathfinder::At(std::vector<nav_msgs::OccupancyGrid>& maps, int x, int y)
{
    signed char val = -1;
    
    if ((x >= 0) && (x < (int)(maps.front().info.width)) && 
        (y >= 0) && (y < (int)(maps.front().info.height)))
    {
        for (std::vector<nav_msgs::OccupancyGrid>::iterator i = maps.begin(); i != maps.end(); i++)
        {   
            val = val < i->data[x + y * i->info.width] ? i->data[x + y * i->info.width] : val;
        }
    }
    else
    {
        return false;
    }

    return val < 50;
}

bool Pathfinder::LOS(std::vector<nav_msgs::OccupancyGrid>& maps, SearchNode* a, SearchNode* b)
{
    double yaw = atan2(b->m_y - a->m_y, b->m_x - a->m_x);
    double xInc = cos(yaw), yInc = sin(yaw);
    double xx = a->m_x, yy = a->m_y;
    
    for (int len = 0; len < hypot(a->m_x - b->m_x, a->m_y - b->m_y); len++)
    {
        if (At(maps, (int)xx, (int)yy))
        {
            xx += xInc;
            yy += yInc;
        }
        else
        {
            return false;
        }
    }
    
    return true;
}
