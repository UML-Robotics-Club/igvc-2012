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

# Robot Constants
AXLE_LENGTH = 0.6604
WHEEL_RADIUS = 0.12065 # 9.5in diameter wheels
WHEEL_CIRC = 2.0 * math.pi * WHEEL_RADIUS
TWO_PI_TICKS = 5000

signal.signal(signal.SIGINT, lambda signum, stack_frame: exit(0))
rospy.init_node("odom")
br = tf.TransformBroadcaster()
listener = tf.TransformListener()

prev_time_stamp = rospy.Time.now()
#last_accel = (0,start_time)
#accel_lock = threading.Lock()
left_filter = kalman.Kalman(2,0.1, 10)
right_filter = kalman.Kalman(2,0.1, 10)

def init_filter(kal):
    dt = 0.1
    state = np.zeros((2,1))
    kal.C = np.array([[1,0]])
    update_filter(kal, dt)

def update_filter(kal, dt):
    kal.A = np.array([[1.0, dt],
                      [0.0, 1.0]])
    kal.B = np.array([[dt**2/2.0],
                      [dt]])
    
#def handle_OS5000_msg(msg):
#    global last_accel, accel_lock
#    accel_lock.acquire()
#    last_accel = (msg.accel, msg.header.stamp)
#    accel_lock.release()

def handle_encoder_msg(msg):
    global accel_lock, prev_time_stamp
    global WHEEL_CIRC, TWO_PI_TICKS
    global br

    # todo: use accel in kalman filter
    #accel_lock.acquire()
    #accel, accel_time = copy.copy(last_accel)
    #accel_lock.release()
    
    dt = (msg.header.stamp - prev_time_stamp).to_sec()
    update_filter(left_filter, dt)
    update_filter(right_filter, dt)
    left_meas_vel = msg.left_ticks * WHEEL_CIRC / TWO_PI_TICKS / dt
    right_meas_vel = msg.right_ticks * WHEEL_CIRC / TWO_PI_TICKS / dt
    accel = 0 # delme
    left_filter.filter(accel, left_meas_vel, dt)
    right_filter.filter(accel, right_meas_vel, dt)
    left_velocity = left_filter.x[1,0]
    right_velocity = right_filter.x[1,0]

    #pos, orientation = motion_model(left_velocity, right_velocity, dt)
    pos, orientation = motion_model(left_meas_vel, right_meas_vel, dt)
    br.sendTransform(pos, orientation, rospy.Time.now(), "/odom", "/gps")

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
        qx = math.cos(theta_p/2)
        qy = 0
        qz = 0
        qw = math.sin(theta_p/2)
    return ((x,y,0), (qx, qy, qz, qw))
    

if __name__ == '__main__':
    init_filter(left_filter)
    init_filter(right_filter)
    #rospy.Subscriber("OS5000", OS5000, handle_OS5000_msg)
    rospy.Subscriber("/robot/encoder_ticks", EncoderStamped, handle_encoder_msg)
    rospy.spin()
