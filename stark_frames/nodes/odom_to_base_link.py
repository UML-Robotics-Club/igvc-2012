#!/usr/bin/python

import roslib
roslib.load_manifest('stark_frames')
import rospy
import tf
import threading
import signal
import math
import numpy as np
import numpy
import copy
import kalman
from stark_driver.msg import EncoderStamped
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
# Robot Constants
AXLE_LENGTH = 0.6604
WHEEL_RADIUS = 0.12065 # 9.5in diameter wheels
WHEEL_CIRC = 2.0 * math.pi * WHEEL_RADIUS
TWO_PI_TICKS = 6215

signal.signal(signal.SIGINT, lambda signum, stack_frame: exit(0))
rospy.init_node("odom")
br = tf.TransformBroadcaster()
listener = tf.TransformListener()

prev_time_stamp = rospy.Time.now()

def handle_encoder_msg(msg):
    global accel_lock, prev_time_stamp
    global WHEEL_CIRC, TWO_PI_TICKS
    global br
    
    dt = (msg.header.stamp - prev_time_stamp).to_sec()
    left_meas_vel = msg.left_ticks * WHEEL_CIRC / TWO_PI_TICKS / dt
    right_meas_vel = msg.right_ticks * WHEEL_CIRC / TWO_PI_TICKS / dt
    pos, orientation = motion_model(left_meas_vel, right_meas_vel, dt)
    br.sendTransform(pos, orientation, rospy.Time.now(), "/base_link", "/odom")
    prev_time_stamp = msg.header.stamp

# Forward kinematics for differential drive robots
def motion_model(left_vel, right_vel, dt):
    global AXLE_LENGTH

    if left_vel == right_vel: # no rotation
        x = 0
        y = left_vel
        qx = 1.0
        qy = 0
        qz = 0
        qw = 0
    else:
        if left_vel == 0: # rotate about left wheel
            R = -AXLE_LENGTH / 2.0
        elif right_vel == 0: # rotate about right wheel
            R = AXLE_LENGTH / 2.0
        elif left_vel == -right_vel: # rotate about center of axle
            R = 0.0
        else:
            R = 0.5 * (left_vel + right_vel)/(right_vel - left_vel)
        icc_x = -R
        icc_y = 0
    
        omega = (right_vel - left_vel) / AXLE_LENGTH
        rot_z = numpy.array([[math.cos(omega*dt), -math.sin(omega*dt), 0],
                             [math.sin(omega*dt), math.cos(omega*dt), 0],
                             [0, 0, 1]])
        trans_origin = numpy.array([[-icc_x],
                                    [-icc_y],
                                    [math.pi/4]])
        trans_back = numpy.array([[icc_x],
                                  [icc_y],
                                  [omega*dt]])
        new_frame = numpy.dot(rot_z, trans_origin) + trans_back
        x = new_frame[0,0]
        y = new_frame[1,0]
        theta_p = new_frame[2,0]
        
        (qx, qy, qz, qw) = quaternion_from_euler(0,0,theta_p)
        #qx = math.cos(theta_p/2)
        #qy = 0
        #qz = 0
        #qw = math.sin(theta_p/2)
    return ((x,y,0), (qx, qy, qz, qw))

if __name__ == '__main__':
    rospy.Subscriber("/robot/encoder_ticks", EncoderStamped, handle_encoder_msg)
    rospy.spin()
