/*
 * This represents the robot's JAUS interface.  This node simply listens to ROS topics
 * and responds to JAUS inquires.
 *
 * Author: Christopher Granz
 * Last modified: 05.02.2012
 */

#include <jaus/mobility/sensors/localposesensor.h>
#include <jaus/mobility/sensors/velocitystatesensor.h>
#include <jaus/mobility/drivers/localwaypointlistdriver.h>
#include <jaus/core/transport/judp.h>
#include <jaus/core/transport/jtcpclient.h>
#include <jaus/core/component.h>
#include <cxutils/keyboard.h>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include "SetLocalPoseCallback.h"
#include "LocalWaypointListDriver.h"

// Uncomment this to make a direction connection to the judges' COP
//#define REAL_JUDGES

// *** Set these to the correct ROS topics to use
const char ROSTopicCmdVelocity[]  = "/robot/cmd_vel";
const char ROSTopicOdometry[]     = "/robot/odom";
const char ROSTopicSetLocalPose[] = "/robot/set_local_pose";
const char ROSTopicSetWaypoints[] = "/robot/waypoints";

// This is our JAUS ID
JAUS::UShort SubsystemID   = 1000;   // ID of our subsystem to use.
JAUS::Byte NodeID          = 1;      // ID of our node to use.
JAUS::Byte ComponentID     = 1;      // ID of the our component.

// This is the judges' JAUS ID
const char JudgeIPString[] = "192.168.1.42";
JAUS::UShort JudgeSubsystemID   = 42;    // ID of our subsystem to use.
JAUS::Byte JudgeNodeID          = 1;     // ID of our node to use.
JAUS::Byte JudgeComponentID     = 1;     // ID of the our component.

/*
 * Globals
 */

// ROS subscribers and publishers
ros::Subscriber subscriberCmdVel;
ros::Subscriber subscriberOdom;
ros::Publisher publisherSetLocalPose;
ros::Publisher publisherSetWaypoints;

// These objects maintain our "state" and the JAUS++ library
//   will respond to JAUS requests.
// These will be updated based on ROS messages.
JAUS::LocalPoseSensor* localPoseSensor;
JAUS::VelocityStateSensor* velocityStateSensor;
LocalWaypointListDriver* localWaypointListDriver;

SetLocalPoseCallback localPoseCallback(&publisherSetLocalPose);

/*
 * Prototypes
 */
int InitializeJAUSComponent(JAUS::Component* c);
JAUS::Subsystem* DiscoverJudgeSubsystem(JAUS::Component* c);
void ROSVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
void ROSOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

/*
 * Main program entry point.
 */
int main(int argc, char* argv[])
{
    std::cout << "Initializing ROS component... ";

    // ROS initialization stuff
    ros::init(argc, argv, "jaus");

    ros::NodeHandle n;

    // subscriptions
    subscriberCmdVel = n.subscribe(ROSTopicCmdVelocity, 1000, ROSVelocityCallback);
    subscriberOdom   = n.subscribe(ROSTopicOdometry, 1000, ROSOdometryCallback);

    // publications
    publisherSetLocalPose = n.advertise<nav_msgs::Odometry>(ROSTopicSetLocalPose, 1000);

    //  -- we just use std_msgs::String for waypoints for now, until we can all agree on
    //     some custom ROS messages for setting local pose, and setting waypoints...
    publisherSetWaypoints = n.advertise<std_msgs::String>(ROSTopicSetWaypoints, 1000);

    std::cout << "OK" << std::endl;

    // JAUS initialization stuff
    JAUS::Component component;
    JAUS::Subsystem* judgeSubsystem;

    std::cout << "Initializing JAUS component... ";

    if (InitializeJAUSComponent(&component) < 0)
    {
        std::cout << "Failed!" << std::endl;
        return -1;
    }

    std::cout << "OK" << std::endl;

    std::cout << "Searching for Judges' COP... ";

    if ((judgeSubsystem = DiscoverJudgeSubsystem(&component)) == NULL)
    {
        std::cout << "Failed!" << std::endl;
        return -1;
    }

    std::cout << "Success (Identification: " << judgeSubsystem->mIdentification << ")" << std::endl;

    /*
     * Set initial states for services
     */

    // Set an initial local pose
    JAUS::LocalPose localPose;
    localPose.SetX(0.0);
    localPose.SetY(0.0);
    localPose.SetTimeStamp(JAUS::Time::GetUtcTime());
    localPoseSensor->SetLocalPose(localPose);

    // Set an initial velocity state
    JAUS::VelocityState velocityState;
    velocityState.SetVelocityX(0.0);
    velocityState.SetYawRate(0.0);
    velocityState.SetTimeStamp(JAUS::Time::GetUtcTime());
    velocityStateSensor->SetVelocityState(velocityState);

    // now loop forever
    std::cout << "READY, We are now listening to ROS topics and updating current JAUS state..." << std::endl;

    JAUS::Management* managementService = (JAUS::Management*)component.GetService(JAUS::Management::Name);
    JAUS::Time::Stamp displayStatusTimeMs = JAUS::Time::GetUtcTimeMs();

    while (true) // (managementService->GetStatus() != JAUS::Management::Status::Shutdown)
    {
        if (CxUtils::GetChar() == 'q')
            break;

        if (JAUS::Time::GetUtcTimeMs() - displayStatusTimeMs > 2000)
        {
            managementService->PrintStatus(); // display a status message
            displayStatusTimeMs = JAUS::Time::GetUtcTimeMs();
        }

        ros::spinOnce(); // process ROS callbacks

        CxUtils::SleepMs(10); // save some CPU time
    }

    component.Shutdown(); // this will take care of cleanup
    return 0;
}

/*
 * Perform all initialize of our JAUS component.
 */
int InitializeJAUSComponent(JAUS::Component* c)
{
    // Setting timeout for control to 0 (disables timeout of control).
    // This is done because the judges' COP does not query for the timeout period
    // and may not send messages to re-acquire/maintain control within the
    // normal 2 second timeout window.
    c->AccessControlService()->SetTimeoutPeriod(0);

    // Velocity Sensor for Velocity State Report (Section IV.9)
    velocityStateSensor = new JAUS::VelocityStateSensor();
    velocityStateSensor->SetSensorUpdateRate(25);   // Updates at 25 Hz (used for periodic events).
    c->AddService(velocityStateSensor);

    // Local Pose Sensor for Position and Orientation reporting (Section IV.10)
    localPoseSensor = new JAUS::LocalPoseSensor();
    localPoseSensor->SetSensorUpdateRate(25);       // Updates at 25 Hz (used for periodic events).
    c->AddService(localPoseSensor);

    // We need a List Manager Service for Local Waypoint List Driver (Section IV.11)
    c->AddService(new JAUS::ListManager());
    localWaypointListDriver = new LocalWaypointListDriver();
    c->AddService(localWaypointListDriver);

    // Set Vehicle Identification Information
    JAUS::Discovery* discoveryService = c->DiscoveryService();

    // set JAUS names for our robot
    discoveryService->SetSubsystemIdentification(JAUS::Subsystem::Vehicle, "Stark");
    discoveryService->SetNodeIdentification("ROS Core System");
    discoveryService->SetComponentIdentification("ROS-JAUS Interface");

    // discovery frequency in Hz -- rules say every 5 seconds
    discoveryService->SetDiscoveryFrequency(1.0/5.0);

    // Now add callbacks to get JAUS messages when they arrive
    // (NOTE: some messages are handled by JAUS++ and we don't need
    //   to do anything special with them)
    c->TransportService()->RegisterCallback(JAUS::SET_LOCAL_POSE, &localPoseCallback);

    // set our JAUS address then initialize everything (start discovery)
    if (c->Initialize(JAUS::Address(SubsystemID, NodeID, ComponentID)) == false)
        return -1;

    return 0;
}

/*
 * Discover the Judges' COP.  Will not return until discovery or failure.
 */
JAUS::Subsystem* DiscoverJudgeSubsystem(JAUS::Component* c)
{
    JAUS::Subsystem* judgeSubsystem = NULL;

#ifdef REAL_JUDGES
    // now that we are initialized, create a direct connection to the Judges' COP
    // (Multicast may not be supported)
    JAUS::JTCPClient* transportService = NULL;
    transportService = (JAUS::JTCPClient*) component->TransportService();
    transportService->AddConnection(JudgeIPString,
      JAUS::Address(JudgeSubsystemID, JudgeNodeID, JudgeComponentID));
#endif

    // get a pointer to our Management Service
    JAUS::Management* managementService = c->ManagementService();
    JAUS::Time::Stamp timeMs = JAUS::Time::GetUtcTimeMs();

    // try to find the judges' COP
    while (managementService->GetStatus() != JAUS::Management::Status::Shutdown)
    {
        CxUtils::SleepMs(10); // save some CPU time

        if (JAUS::Time::GetUtcTimeMs() - timeMs < 1000)
            continue;

        // now look at discovered subsystems (for the judges' COP)
        JAUS::Discovery* discoveryService = c->DiscoveryService();

        JAUS::Subsystem::Map discoveredSubsystems;
        discoveryService->GetSubsystems(discoveredSubsystems);

        JAUS::Subsystem::Map::iterator subsystem;

        // NOTE: the map is indexed by the subsystem number
        for (subsystem = discoveredSubsystems.begin();
             subsystem != discoveredSubsystems.end();
             subsystem++)
        {
            if (subsystem->first == JudgeSubsystemID)
            {
                judgeSubsystem = subsystem->second;
                break;
            }
        }

        JAUS::Subsystem::DeleteSubsystemMap(discoveredSubsystems);

        timeMs = JAUS::Time::GetUtcTimeMs();

        if (judgeSubsystem != NULL)
            break;
    }

    return judgeSubsystem;
}

/*
 * ROS callbacks start here...
 */
void ROSVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    JAUS::VelocityState velocityState;
    velocityState.SetVelocityX(msg->linear.x);
    velocityState.SetYawRate(msg->angular.z);
    velocityState.SetTimeStamp(JAUS::Time::GetUtcTime());
    // Save the data to the service
    velocityStateSensor->SetVelocityState(velocityState);

//    ROS_INFO("Got ROS data: [%f]", msg->linear.x);
}

void ROSOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    JAUS::LocalPose localPose;
    localPose.SetX(msg->pose.pose.position.x);
    localPose.SetY(msg->pose.pose.position.y);
    localPose.SetTimeStamp(JAUS::Time::GetUtcTime());
    localPoseSensor->SetLocalPose(localPose);
}

/*
 * JAUS callbacks start here...
 */


/*
 * End of jaus.cpp
 */

