#!/usr/bin/python

import roslib
roslib.load_manifest('stark_frames')
import rospy
import tf
from geometry_msgs.msg import PoseStamped

def handle_gps_msg(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    qx = msg.pose.orientation.x
    qy = msg.pose.orientation.y
    qz = msg.pose.orientation.z
    qw = msg.pose.orientation.w

    br = tf.TransformBroadcaster()
    br.sendTransform((x,y,0), 
                     (qx,qy,qz,qw),
                     rospy.Time.now(),
                     "/gps",
                     "/map")

if __name__ == '__main__':
    rospy.init_node("map_node")
    rospy.Subscriber("imu_pose", PoseStamped, handle_gps_msg)
    rospy.spin()
