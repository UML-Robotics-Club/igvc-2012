#include "transformHelper.hpp"

static tf::TransformListener* tfListener;

void initTfHelper()
{
    tfListener = new tf::TransformListener();
}

void getTransform(std::string to, std::string from, ros::Time time, ros::Duration duration, double& x, double& y, double& yaw)
{
    tf::StampedTransform transform;
    try
    {
        tfListener->waitForTransform(to, from, time, duration);
        tfListener->lookupTransform(to, from, time, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    
    double r, p;
    
    btMatrix3x3(transform.getRotation()).getRPY(r, p, yaw);
    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
}