
#include <unistd.h>
#include <libguile.h>
#include <math.h>
#include <stdint.h>
#include <vector>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/ColorRGBA.h"

#include "woah_scheme/transform.hh"

using namespace nav_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace visualization_msgs;
using namespace std_msgs;

static ros::Publisher* cmd_vel;
static ros::Publisher* vis_pub;

static PoseStamped goal;
static bool final_wp;

static SCM woah;

static uint16_t next_line_id = 0;

void
draw_line(const char* frame_id, double xx, double yy, double tt, 
    double ss, double rr, double gg, double bb)
{
    Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp    = ros::Time();
    marker.ns = "woah_scheme";
    marker.id = next_line_id++;
    marker.type = Marker::LINE_LIST;
    marker.action = Marker::ADD;

    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);
    marker.pose.orientation.x = quat.getX();
    marker.pose.orientation.y = quat.getY();
    marker.pose.orientation.z = quat.getZ();
    marker.pose.orientation.w = quat.getW();

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.a = 1.0;

    Point p0; 
    p0.x = xx; p0.y = yy; p0.z = 0.5;
    marker.points.push_back(p0);
    Point p1; p1.z = 0.5;
    p1.x = xx + ss * cos(tt); p1.y = yy + ss * sin(tt);
    marker.points.push_back(p1);

    ColorRGBA c1; c1.a = 1.0; 
    c1.r = rr; c1.g = gg; c1.b = bb;
    marker.colors.push_back(c1);
    marker.colors.push_back(c1);

    vis_pub->publish(marker);
}

void
got_path(const Path::ConstPtr& msg)
{
    if (msg->poses.size() == 0)
        return;

    if (msg->poses.size() == 1)
        final_wp = true;
    else
        final_wp = false;

    goal = msg->poses.back();
}

void
got_scan(const LaserScan::ConstPtr& msg)
{
    next_line_id = 0;

    // Determine current position.
    double xx, yy, tt;
    getTransform(goal.header.frame_id, msg->header.frame_id, 
        msg->header.stamp, ros::Duration(1,0), xx, yy, tt);

    double dy = goal.pose.position.y - yy;
    double dx = goal.pose.position.x - xx;

    double goal_alpha = atan2(dy, dx) - tt;
    double goal_dist  = hypot(dx, dy);

    // Build list of pairs.
    SCM ranges = SCM_EOL;
    double angle = msg->angle_min;
    for (uint64_t ii = 0; ii < msg->ranges.size(); ++ii) {
        double theta = angle + ii * msg->angle_increment;
        double range = msg->ranges[ii];
        SCM cell = scm_cons(scm_from_double(theta), scm_from_double(range));
        ranges = scm_cons(cell, ranges);
    }

    double speed = 0.0;
    double rotsp = 0.0;

    draw_line("base_link", 0.0, 0.0, goal_alpha, 3.0, 0.0, 1.0, 1.0);

    if (goal_dist > 0.3) {
        SCM cmd_pair = scm_call_4(woah, scm_from_double(goal_alpha), scm_from_double(goal_dist), 
            scm_from_bool(final_wp), ranges);

        speed = scm_to_double(scm_car(cmd_pair));
        rotsp = scm_to_double(scm_cdr(cmd_pair));
    }

    // Send the motor command.
    Twist cmd;
    cmd.linear.x  = speed;
    cmd.angular.z = rotsp;
    cmd_vel->publish(cmd);
}

int
main(int argc, char* argv[])
{
    if (chdir(PACKAGE_PATH) != 0) {
        fprintf(stderr, "Cannot chdir to %s", PACKAGE_PATH);
        return 1;
    }

    scm_init_guile();
    scm_c_eval_string("(load \"src/woah.scm\")");
    woah = scm_variable_ref(scm_c_lookup("woah"));

    ros::init(argc, argv, "woah_scm");
    ros::NodeHandle node;

    ros::Publisher cv_ = node.advertise<Twist>("cmd_vel", 0);
    cmd_vel = &cv_;
    ros::Publisher vp_ = node.advertise<Marker>("visualization_marker", 0);
    vis_pub = &vp_;

    ros::Subscriber path_sub = node.subscribe("path", 1, got_path);
    ros::Subscriber scan_sub = node.subscribe("base_scan", 1, got_scan);
    
    ros::spin();
    return 0;
}
