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
    initTfHelper();
    
    double xx, yy, dc;
    
    getTransform("/map", "/base_link", ros::Time::now(), ros::Duration(5.0), xx, yy, dc);
    
    SetTarget(xx, yy);
 
    //m_debug = nh.advertise<visualization_msgs::MarkerArray>("debug", 1);
    
    ros::param::param<double>("~pathfinderTolerance", m_tolerance, 1.5);
}

void Pathfinder::SetTarget(double xt, double yt)
{
    m_xt = xt;
    m_yt = yt;
}

nav_msgs::Path Pathfinder::MakePath(nav_msgs::OccupancyGrid& map)
{
    SearchNodeCmp::SetTarget(m_xt / map.info.resolution, m_yt / map.info.resolution);
    std::priority_queue<SearchNode*, std::vector<SearchNode*>, SearchNodeCmp> queue;
    
    tf::StampedTransform transform;
    try
    {
        ros::Time stamp = ros::Time::now();
        
        m_tfListener.waitForTransform("/map", "/base_link", stamp, ros::Duration(1.0));
        m_tfListener.lookupTransform("/map", "base_link", stamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    
    double xxd, yyd, dc;
    
    getTransform("/map", "/base_link", ros::Time::now(), ros::Duration(1.0), xxd, yyd, dc);
    
    int xx = xxd / map.info.resolution;
    int yy = yyd / map.info.resolution;
    
    m_searchmap = new SearchNode[map.info.height * map.info.width];
    for (unsigned int y = 0; y < map.info.height; ++y)
    {
        for (unsigned int x = 0; x < map.info.width; ++x)
        {
            m_searchmap[y * map.info.width + x].m_x = x;
            m_searchmap[y * map.info.width + x].m_y = y;
        }
    }
    
    /* Should get current vel */
    queue.push(&(m_searchmap[yy * map.info.width + xx]));
    queue.top()->m_cost = 0;
    queue.top()->m_parent = HOME_NODE;
    
    ROS_INFO("START (FL): %lf, %lf", xxd, yyd);
    ROS_INFO("TARGE (FL): %lf, %lf", m_xt, m_yt);
    
    ROS_INFO("NODE DETAILS: %d %d %lf", 
             m_searchmap[yy * map.info.width + xx].m_x,
             m_searchmap[yy * map.info.width + xx].m_y,
             m_searchmap[yy * map.info.width + xx].m_cost);
    
    /*visualization_msgs::MarkerArray markers;
    
    {
        //Red Lines
        visualization_msgs::Marker mark;
        
        mark.header.stamp = ros::Time::now();
        mark.header.frame_id = "/map";
        mark.action = visualization_msgs::Marker::ADD;
        mark.ns = "Failed_LOS";
        mark.id = 1;
        mark.type = visualization_msgs::Marker::LINE_LIST;
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
        markers.markers.push_back(mark);
    }
    
    {
        //Green Lines
        visualization_msgs::Marker mark;
        
        mark.header.stamp = ros::Time::now();
        mark.header.frame_id = "/map";
        mark.action = visualization_msgs::Marker::ADD;
        mark.ns = "Passed_LOS";
        mark.id = 1;
        mark.type = visualization_msgs::Marker::LINE_LIST;
        mark.lifetime = ros::Duration();
        
        mark.scale.x = 0.01;
        
        mark.pose.orientation.x = 0.0;
        mark.pose.orientation.y = 0.0;
        mark.pose.orientation.z = 0.0;
        mark.pose.orientation.w = 1.0;
        
        mark.color.r = 0.0;
        mark.color.g = 1.0;
        mark.color.b = 0.0;
        mark.color.a = 1.0;   
        markers.markers.push_back(mark);
    }*/
    
    while (!queue.empty() && queue.size() < 500000)
    {
        SearchNode* nxt;
        SearchNode* cur = queue.top();
        queue.pop();
        
        //markers.markers.push_back(DebugArrow(cur, map.info.resolution));
        
        //ROS_INFO("NODE(%d): %lf, %lf", queue.size(), cur->m_x, cur->m_y);
        
        if (hypot(cur->m_x - m_xt / map.info.resolution, 
                  cur->m_y - m_yt / map.info.resolution) <= m_tolerance)
        {
            //Done - Assemble Path
            
            m_lastPath = nav_msgs::Path();
            m_lastPath.header.frame_id = "/map";
            
            while (cur != HOME_NODE)
            {
                geometry_msgs::PoseStamped pose;
                
                pose.header.frame_id = "/map";
                pose.pose.position.x = cur->m_x * map.info.resolution;
                pose.pose.position.y = cur->m_y * map.info.resolution;
                
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
            
            //m_debug.publish(markers);
            
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
                        
                        if (At(map, nx, ny)) 
                        {
                            nxt = &(m_searchmap[ny * map.info.width + nx]);
                            
                            if (nxt->m_parent == NULL || (nxt->m_parent != HOME_NODE && nxt->m_parent->m_cost > cur->m_cost))
                            {
                                
                                if (cur->m_parent > HOME_NODE && LOS(map, cur->m_parent, nxt))
                                {
                                    nxt->m_cost = cur->m_parent->m_cost + hypot(nxt->m_x - cur->m_parent->m_x, 
                                                                                nxt->m_y - cur->m_parent->m_y);
                                    nxt->m_parent = cur->m_parent;
                                    
                                    //Viz Passed checks
                                    //GreenDebugLine(markers, cur->m_parent, nxt, map.info.resolution);
                                    
                                }
                                else
                                {
                                    nxt->m_cost = cur->m_cost + hypot(nxt->m_x - cur->m_x, 
                                                                      nxt->m_y - cur->m_y);
                                    nxt->m_parent = cur;
                                    
                                    if (cur->m_parent > HOME_NODE)
                                    {
                                        //Viz Failed checks
                                        //RedDebugLine(markers, cur->m_parent, nxt, map.info.resolution);
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
    
    //m_debug.publish(markers);
    
    delete[] m_searchmap;
    
    return GetLastPath();
}

nav_msgs::Path Pathfinder::GetLastPath()
{
    return m_lastPath;
}

bool Pathfinder::At(nav_msgs::OccupancyGrid& map, int x, int y)
{
    if ((x >= 0) && (x < (int)(map.info.width)) && 
        (y >= 0) && (y < (int)(map.info.height)))
    {
        return map.data[x + y * map.info.width] < 50;
    }
    
    return false;
}

bool Pathfinder::LOS(nav_msgs::OccupancyGrid& map, SearchNode* a, SearchNode* b)
{
    double yaw = atan2(b->m_y - a->m_y, b->m_x - a->m_x);
    double xInc = cos(yaw), yInc = sin(yaw);
    double xx = a->m_x, yy = a->m_y;
    
    for (int len = 0; len < hypot(a->m_x - b->m_x, a->m_y - b->m_y); len++)
    {
        if (At(map, (int)xx, (int)yy))
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

void Pathfinder::RedDebugLine(visualization_msgs::MarkerArray& markers, SearchNode* a, SearchNode* b, double res)
{   
    markers.markers[0].points.push_back(geometry_msgs::Point());
    markers.markers[0].points.back().x = a->m_x * res;
    markers.markers[0].points.back().y = a->m_y * res;
    markers.markers[0].points.back().z = a->m_cost / 50.0;
    
    markers.markers[0].points.push_back(geometry_msgs::Point());
    markers.markers[0].points.back().x = b->m_x * res;
    markers.markers[0].points.back().y = b->m_y * res;
    markers.markers[0].points.back().z = b->m_cost / 50.0;
    
    markers.markers[0].colors.push_back(std_msgs::ColorRGBA());
    markers.markers[0].colors.back().r = 1.0;
    markers.markers[0].colors.back().g = 0.0;
    markers.markers[0].colors.back().b = 0.0;
    markers.markers[0].colors.back().a = 1.0;
    
    markers.markers[0].colors.push_back(std_msgs::ColorRGBA());
    markers.markers[0].colors.back().r = 1.0;
    markers.markers[0].colors.back().g = 0.0;
    markers.markers[0].colors.back().b = 0.0;
    markers.markers[0].colors.back().a = 1.0;
}

void Pathfinder::GreenDebugLine(visualization_msgs::MarkerArray& markers, SearchNode* a, SearchNode* b, double res)
{
    markers.markers[1].points.push_back(geometry_msgs::Point());
    markers.markers[1].points.back().x = a->m_x * res;
    markers.markers[1].points.back().y = a->m_y * res;
    markers.markers[1].points.back().z = a->m_cost / 50.0;
    
    markers.markers[1].points.push_back(geometry_msgs::Point());
    markers.markers[1].points.back().x = b->m_x * res;
    markers.markers[1].points.back().y = b->m_y * res;
    markers.markers[1].points.back().z = b->m_cost / 50.0;
    
    markers.markers[1].colors.push_back(std_msgs::ColorRGBA());
    markers.markers[1].colors.back().r = 0.0;
    markers.markers[1].colors.back().g = 1.0;
    markers.markers[1].colors.back().b = 0.0;
    markers.markers[1].colors.back().a = 1.0;
    
    markers.markers[1].colors.push_back(std_msgs::ColorRGBA());
    markers.markers[1].colors.back().r = 0.0;
    markers.markers[1].colors.back().g = 1.0;
    markers.markers[1].colors.back().b = 0.0;
    markers.markers[1].colors.back().a = 1.0;
}

visualization_msgs::Marker Pathfinder::DebugArrow(SearchNode* a, double res)
{
    visualization_msgs::Marker mark;
                                        
    mark.header.stamp = ros::Time::now();
    mark.header.frame_id = "/map";
    mark.action = visualization_msgs::Marker::ADD;
    mark.ns = "Expansion";
    mark.id = a->m_x + 500 * a->m_y;
    mark.type = visualization_msgs::Marker::ARROW;
    mark.lifetime = ros::Duration(1.0);
    
    mark.scale.x = 0.25;
    mark.scale.y = 0.25;
    mark.scale.z = res / 1.5;
    
    double yaw = 0;
    
    if (a->m_parent > HOME_NODE)
    {
        yaw = atan2(a->m_y - a->m_parent->m_y,
                    a->m_x - a->m_parent->m_x);
    }
    
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    
    mark.pose.orientation.x = quat.getX();
    mark.pose.orientation.y = quat.getY();
    mark.pose.orientation.z = quat.getZ();
    mark.pose.orientation.w = quat.getW();
    
    mark.pose.position.x = a->m_x * res;
    mark.pose.position.y = a->m_y * res;
    
    mark.color.r = 0.0;
    mark.color.g = 0.0;
    mark.color.b = 1.0;
    mark.color.a = 1.0;
    
    return mark;
}
