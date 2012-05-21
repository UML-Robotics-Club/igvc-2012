
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <string>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "MotorDriver.hh"
#include "croak.hh"

MotorDriver* motor;

void
got_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
    cout << "Speed, Turn: " << msg->linear.x 
         << ", " << msg->angular.z << endl;
    motor->SetSpeed(msg->linear.x, msg->angular.z);
}

int
main(int argc, char* argv[])
{
    std::string device;

    ros::init(argc, argv, "stark_driver");
    ros::NodeHandle node("~");

    node.getParam("device", device);
    
    if (device == "") {
        croak("No device specified");
    }
    
    cout << "Node stark_driver starting up." << endl;

    motor = new MotorDriver(device);
    ros::Subscriber sub = node.subscribe("cmd_vel", 1000, got_cmd_vel);

    cout << "Node stark_driver listening for messages." << endl;
    ros::spin();

    delete motor;
    
    return 0;
}
