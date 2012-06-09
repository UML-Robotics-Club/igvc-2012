#!/usr/bin/python

import roslib
roslib.load_manifest('stark_frames')
import rospy
import tf
from geometry_msgs.msg import Pose2D
import math

def handle_pose_msg(msg):
    x = msg.x
    y = msg.y
    qx = math.cos(msg.theta/2.0)
    qy = 0
    qz = 0
    qw = math.sin(msg.theta/2.0)
    br = tf.TransformBroadcaster()
    br.sendTransform((x,y,0), 
                     (qx,qy,qz,qw),
                     rospy.Time.now(),
                     "/map",
                     "/gps")

if __name__ == '__main__':
    rospy.init_node("map_node")
    rospy.Subscriber("/robot/pose2d", Pose2D, handle_pose_msg)
    rospy.spin()
