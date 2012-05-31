#!/usr/bin/env python
import roslib
roslib.load_manifest('kec_line')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('camera_display')
bridge = CvBridge()
cv.NamedWindow('input',1)

def got_frame(frame):
    image = bridge.imgmsg_to_cv(frame, "bgr8")
    cv.ShowImage('input', image)
    cv.WaitKey(10)

#cv.DestroyAllWindows()
rospy.Subscriber('cam', Image, got_frame, queue_size=1)
rospy.spin()

