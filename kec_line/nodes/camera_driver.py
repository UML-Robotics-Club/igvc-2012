#!/usr/bin/env python
import roslib
roslib.load_manifest('kec_line')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



pub = rospy.Publisher('cam', Image)
rospy.init_node('camera_driver')
cam = cv.CaptureFromCAM(201)
bridge = CvBridge()

#r = rospy.Rate(1) #run the camera at 10Hz to mimic laser
while not rospy.is_shutdown():
    frame = cv.QueryFrame(cam)
    print frame.width, frame.height
    pub.publish(bridge.cv_to_imgmsg(frame, "bgr8"))
    #r.sleep()
    #cv.WaitKey(10)

