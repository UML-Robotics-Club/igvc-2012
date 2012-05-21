#include <queue>
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/transform_listener.h"

class SearchNode
{
public:
    
    double m_x, m_y;
    double m_cost;
    SearchNode* m_parent;
    
    SearchNode();
    SearchNode(double x, double y, double cost, SearchNode* parent);
};

class SearchNodeCmp
{
public:
    static double m_xt, m_yt;
    
    bool operator()(const SearchNode* a, const SearchNode* b);
    
    static void SetTarget(double xt, double yt);
};

class Pathfinder
{
public:
    
    Pathfinder(ros::NodeHandle& nh);
    
    void SetTarget(double xt, double yt);
    nav_msgs::Path MakePath(double timeout, std::vector<nav_msgs::OccupancyGrid>& maps);
    nav_msgs::Path GetLastPath();
    
protected:
    
    ros::NodeHandle &m_nh;
    ros::Publisher m_poses;
    
    bool At(std::vector<nav_msgs::OccupancyGrid>& maps, int x, int y);
    bool LOS(std::vector<nav_msgs::OccupancyGrid>& maps, SearchNode* a, SearchNode* b);
    
    double m_xt, m_yt, m_tolerance;
    SearchNode *m_searchmap;
    
    tf::TransformListener m_tfListener;
    nav_msgs::Path m_lastPath;
};