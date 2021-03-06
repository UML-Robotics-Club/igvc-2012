#include <queue>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include "std_msgs/ColorRGBA.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_listener.h"

#include "transformHelper.hpp"

#define HOME_NODE (SearchNode*)3

class SearchNode
{
public:
    
    int m_x, m_y;
    double m_cost;
    SearchNode* m_parent;
    
    SearchNode();
    SearchNode(int x, int y, double cost, SearchNode* parent);
};

class SearchNodeCmp
{
public:
    static int m_xt, m_yt;
    
    bool operator()(const SearchNode* a, const SearchNode* b);
    
    static void SetTarget(int xt, int yt);
};

class Pathfinder
{
public:
    
    Pathfinder(ros::NodeHandle& nh);
    
    void SetTarget(double xt, double yt);
    nav_msgs::Path MakePath(nav_msgs::OccupancyGrid& map);
    nav_msgs::Path GetLastPath();
    
protected:
    
    ros::NodeHandle &m_nh;
    ros::Publisher m_debug;
    
    bool At(nav_msgs::OccupancyGrid& map, int x, int y);
    bool LOS(nav_msgs::OccupancyGrid& map, SearchNode* a, SearchNode* b);
    
    void RedDebugLine(visualization_msgs::MarkerArray& markers, SearchNode* a, SearchNode* b, double res);
    void GreenDebugLine(visualization_msgs::MarkerArray& markers, SearchNode* a, SearchNode* b, double res);
    visualization_msgs::Marker DebugArrow(SearchNode* a, double res);
    
    double m_xt, m_yt, m_tolerance;
    SearchNode *m_searchmap;
    
    tf::TransformListener m_tfListener;
    nav_msgs::Path m_lastPath;
};