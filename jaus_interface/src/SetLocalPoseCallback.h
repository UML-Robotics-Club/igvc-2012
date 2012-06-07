/*
 * Author: Christopher Granz
 * Last modified: 05.02.2012
 */

#ifndef __SET_LOCAL_POSE_CALLBACK_H__
#define __SET_LOCAL_POSE_CALLBACK_H__

#include <jaus/mobility/sensors/localposesensor.h>
#include <iostream>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

class SetLocalPoseCallback : public JAUS::Transport::Callback
{
private:
    ros::Publisher* pub;

public:
    SetLocalPoseCallback(ros::Publisher* pubSetLocalPose)
    {
        pub = pubSetLocalPose;
    }

    ~SetLocalPoseCallback() {}
    virtual void ProcessMessage(const JAUS::Message* message)
    {
        const JAUS::SetLocalPose* setLocalPose = dynamic_cast<const JAUS::SetLocalPose*>(message);

        if (setLocalPose)
        {
            setLocalPose->Print();
            double x = setLocalPose->GetX();
            double y = setLocalPose->GetY();
            double yaw = setLocalPose->GetYaw();
            printf("Got Set Local Pose JAUS message (x=%f,y=%f,yaw=%f)\n",x,y,yaw);

            // Now publish it to our ROS topic
            nav_msgs::Odometry msg;
            msg.pose.pose.position.x = x;
            msg.pose.pose.position.y = y;
            msg.twist.twist.angular.z = yaw;

            if (pub)
                pub->publish(msg);
        }
    }
};

#endif

/*
 * End of SetLocalPoseCallback.cpp
 */

