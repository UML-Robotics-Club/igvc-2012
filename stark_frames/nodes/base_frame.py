#!/usr/bin/python

import roslib
roslib.load_manifest('stark_frames')
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node("base_link_node")
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        br.sendTransform((0,0.381,0),
                         (1.0, 0, 0, 0),
                         rospy.Time.now(),
                         "base_link",
                         "odom")
        rospy.sleep(1.0)
