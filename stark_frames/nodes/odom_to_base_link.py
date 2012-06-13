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
from geometry_msgs.msg import QuaternionStamped
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

# Robot Constants
AXLE_LENGTH = 0.58 # actual 0.64
WHEEL_RADIUS = 0.127 # 9.5in diameter wheels
WHEEL_CIRC = 2.0 * math.pi * WHEEL_RADIUS
TWO_PI_TICKS = 98917 # actual ~62127 (i think)

signal.signal(signal.SIGINT, lambda signum, stack_frame: exit(0))
rospy.init_node("odom")
br = tf.TransformBroadcaster()
listener = tf.TransformListener()

prev_time_stamp = rospy.Time.now()
gps_stamp = prev_time_stamp
reset_model = False
x_pos = 0
y_pos = 0
theta = math.pi/2.0 
compass_heading = None

unhappy_pub = rospy.Publisher('/robot/bili_tf', PoseStamped)

def handle_compass_msg(msg):
    global compass_heading
    (roll,pitch,yaw) = euler_from_quaternion((msg.quaternion.x,
                                              msg.quaternion.y,
                                              msg.quaternion.z,
                                              msg.quaternion.w))
    compass_heading = yaw

def handle_gps_msg(msg):
    global reset_model, gps_stamp
    gps_stamp = msg.header.stamp
    reset_model = False #set to true if you want to reset 

def handle_encoder_msg(msg):
    global unhapy_pub
    global prev_time_stamp
    global WHEEL_CIRC, TWO_PI_TICKS
    global br
    global reset_model, gps_stamp
    global x_pos, y_pos, theta
    global compass_heading
    
    timestamp = msg.header.stamp
    if reset_model:
        x_pos = 0.0
        y_pos = 0.0
        theta = math.pi/2.0
        timestamp = gps_stamp
        reset_model = False
    
    dt = (msg.header.stamp - prev_time_stamp).to_sec()
    left_meas_vel = msg.left_ticks * WHEEL_CIRC / TWO_PI_TICKS / dt
    right_meas_vel = msg.right_ticks * WHEEL_CIRC / TWO_PI_TICKS / dt
    x_pos, y_pos, theta = motion_model(left_meas_vel, right_meas_vel,
                                       x_pos, y_pos, theta, dt)
    pos = (y_pos, -x_pos, 0)
    if(abs(left_meas_vel + right_meas_vel) < 0.2 and compass_heading is not None):
        (qx, qy, qz, qw) = quaternion_from_euler(0,0,compass_heading)
        theta = compass_heading + math.pi/2.0
    else:
        (qx, qy, qz, qw) = quaternion_from_euler(0,0,theta-math.pi/2.0)
    orientation = (qx, qy, qz, qw)
    br.sendTransform(pos, orientation, timestamp, "/base_link", "/odom")
    unhappy_pose = PoseStamped()
    unhappy_pose.header.stamp = timestamp
    unhappy_pose.pose.position.x = pos[0]
    unhappy_pose.pose.position.y = pos[1]
    unhappy_pose.pose.position.z = pos[2]
    unhappy_pose.pose.orientation.x = qx
    unhappy_pose.pose.orientation.y = qy
    unhappy_pose.pose.orientation.z = qz
    unhappy_pose.pose.orientation.w = qw
    unhappy_pub.publish(unhappy_pose)
    prev_time_stamp = msg.header.stamp

# Forward kinematics for differential drive robots
def motion_model(left_vel, right_vel, x_pos, y_pos, theta, dt):
    global AXLE_LENGTH

    if left_vel == right_vel: # no rotation
        y_pos += left_vel
    else:
        if left_vel == 0: # rotate about left wheel
            R = -AXLE_LENGTH / 2.0
        elif right_vel == 0: # rotate about right wheel
            R = AXLE_LENGTH / 2.0
        elif left_vel == -right_vel: # rotate about center of axle
            R = 0.0
        else:
            R = 0.5 * (left_vel + right_vel)/(right_vel - left_vel)
        icc_x = x_pos - R*math.sin(theta)
        icc_y = y_pos + R*math.cos(theta)
    
        omega = (right_vel - left_vel) / AXLE_LENGTH
        rot_z = numpy.array([[math.cos(omega*dt), -math.sin(omega*dt), 0],
                             [math.sin(omega*dt), math.cos(omega*dt), 0],
                             [0, 0, 1]])
        trans_origin = numpy.array([[x_pos - icc_x],
                                    [y_pos - icc_y],
                                    [theta]])
        trans_back = numpy.array([[icc_x],
                                  [icc_y],
                                  [omega*dt]])
        new_frame = numpy.dot(rot_z, trans_origin) + trans_back
        x_pos = new_frame[0,0]
        y_pos = new_frame[1,0]
        theta = new_frame[2,0]
        #y = -new_frame[0,0]
        #x = new_frame[1,0]
        #theta_p = new_frame[2,0] - math.pi/2.0
    return (x_pos, y_pos, theta)

if __name__ == '__main__':
    rospy.Subscriber("/robot/encoder_ticks", EncoderStamped, handle_encoder_msg)
    rospy.Subscriber("/robot/pose2d", PoseStamped, handle_gps_msg)
    rospy.Subscriber("/robot/orientation", QuaternionStamped, handle_compass_msg)
    
    rospy.spin()
    
