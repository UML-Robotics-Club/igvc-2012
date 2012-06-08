
#include <unistd.h>
#include <libguile.h>
#include <math.h>
#include <stdint.h>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "woah_scheme/transform.hh"

using namespace nav_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;

static ros::Publisher* cmd_vel;

static PoseStamped goal;
static bool final_wp;

static SCM woah;

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
    // Determine current position.
    double xx, yy, tt;
    getTransform(goal.header.frame_id, msg->header.frame_id, 
        msg->header.stamp, ros::Duration(1,0), xx, yy, tt);

    double dy = goal.pose.position.y - yy;
    double dx = goal.pose.position.x - xx;

    double goal_alpha = atan2(dy, dx);
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

    SCM cmd_pair = scm_call_4(woah, scm_from_double(goal_alpha), scm_from_double(goal_dist), 
        scm_from_bool(final_wp), ranges);

    double speed = scm_to_double(scm_car(cmd_pair));
    double rotsp = scm_to_double(scm_cdr(cmd_pair));

    // Send the motor command.
    Twist cmd;
    cmd.linear.x  = speed;
    cmd.angular.z = rotsp;
    cmd_vel->publish(cmd);
}



int
main(int argc, char* argv[])
{
    chdir(PACKAGE_PATH);

    scm_init_guile();
    scm_c_eval_string("(load \"src/woah.scm\")");
    woah = scm_variable_ref(scm_c_lookup("woah"));

    ros::init(argc, argv, "woah_scm");
    ros::NodeHandle node;

    ros::Publisher cv_ = node.advertise<Twist>("cmd_vel", 1);
    cmd_vel = &cv_;

    ros::Subscriber path_sub = node.subscribe("path", 1, got_path);
    ros::Subscriber scan_sub = node.subscribe("base_scan", 1, got_scan);
    
    ros::spin();
    return 0;
}
