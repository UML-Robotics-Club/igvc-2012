#!/usr/bin/python

import roslib
roslib.load_manifest('stark_frames')
import rospy
import tf
from geometry_msgs.msg import PoseStamped
import math
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node("map_node")
    good_msg = False
    while(not good_msg):
        msg = rospy.wait_for_message("/robot/pose2d", PoseStamped)
        if not math.isnan(msg.pose.position.x) and not math.isnan(msg.pose.position.y):
            good_msg = True
            pos = (msg.pose.position.x-100, msg.pose.position.y-100, 0)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        br = tf.TransformBroadcaster()
        br.sendTransform(pos, (0, 0, 0, 1), rospy.Time.now(), "/map", "/gps")
        rate.sleep()
