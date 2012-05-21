#!/usr/bin/env python
import roslib; roslib.load_manifest('uml_sim_gps')
import rospy
import math
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from time import time

last = time()
callback = None
delay = 0.0

def got_ground_truth(odom):
    global last

    now = time()

    if (last + delay) <= now:
        last = now
        xx = -odom.pose.pose.position.y
        yy = odom.pose.pose.position.x

        quat = odom.pose.pose.orientation
        (_, _, angle) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        callback(xx, yy, angle + math.pi / 2.0, odom.header.stamp)

def sim_gps(CONFIG, cb):
    global callback
    callback = cb

    global delay
    delay = CONFIG['UPDATE_DELAY']

    rospy.Subscriber("robot/base_pose_ground_truth", Odometry, got_ground_truth)
    rospy.spin()
