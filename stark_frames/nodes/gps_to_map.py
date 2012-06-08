#!/usr/bin/python

import roslib
roslib.load_manifest('stark_frames')
import rospy
import tf
from geometry_msgs.msg import Pose2D
import math

if __name__ == '__main__':
    rospy.init_node("map_node")
    good_msg = False
    while(not good_msg):
        msg = rospy.wait_for_message("/robot/pose2d", Pose2D)
        if not math.isnan(msg.x) and not math.isnan(msg.y) and not math.isnan(msg.theta):
            good_msg = True
            pos = (msg.x-100, msg.y-100, 0)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        br = tf.TransformBroadcaster()
        br.sendTransform(pos, (0,0,0,1), rospy.Time.now(), "/map", "/gps")
        rate.sleep()
