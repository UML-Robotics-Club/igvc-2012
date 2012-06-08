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
    while(!good_msg):
        msg = wait_for_message("/robot/pose2d", Pose2D)
        if not math.isnan(msg.x) && not math.isnan(msg.y) && not math.isnan(msg.theta):
            good_msg = True
            pos = (msg.x, msg.y, 0)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        br = tf.TransformBroadcaster()
        br.sendTransform(pos, (1,0,0,0), rospy.Time.now(), "/gps", "/map")
        rate.sleep()
