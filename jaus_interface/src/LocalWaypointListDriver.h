/*
 * This class implements any additional functional on top of what is built into JAUS++.
 *
 * Author: Christopher Granz
 * Last modified: 05.02.2012
 */

#ifndef __LOCAL_WAYPOINT_LIST_DRIVER_H__
#define __LOCAL_WAYPOINT_LIST_DRIVER_H__

#include <jaus/mobility/sensors/localposesensor.h>
#include <jaus/mobility/sensors/velocitystatesensor.h>
#include <jaus/mobility/drivers/localwaypointlistdriver.h>
#include <jaus/core/transport/judp.h>
#include <jaus/core/transport/jtcpclient.h>
#include <jaus/core/component.h>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

class LocalWaypointListDriver : public JAUS::LocalWaypointListDriver
{
public:
    LocalWaypointListDriver() {}
    ~LocalWaypointListDriver() {}
};

#endif

/*
 * End of LocalWaypointListDriver.cpp
 */

