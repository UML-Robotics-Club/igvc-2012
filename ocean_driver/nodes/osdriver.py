#!/usr/bin/env python
import roslib; roslib.load_manifest('ocean_driver')
import rospy
import math
import serial
import time
from geometry_msgs.msg import QuaternionStamped
from tf.transformations import quaternion_from_euler

rospy.init_node('os5000')
pub = rospy.Publisher('/robot/orientation', QuaternionStamped)

ser = serial.Serial('/dev/ttyOCEAN', 19200)

def main():
    global ser, pub
    while 1:
        msg = ser.readline().split(",");
        degrees = float(msg[1])
        radians = degrees * math.pi / 180.0 + math.pi / 2.0 # east is 0
        if radians > math.pi:
            radians = -(radians - math.pi)
        
        (qx, qy, qz, qw) = quaternion_from_euler(0,0,radians)
	msg = QuaternionStamped()
	msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        msg.quaternion.x = qx
        msg.quaternion.y = qy
        msg.quaternion.z = qz
        msg.quaternion.w = qw

        pub.publish(msg)
        ser.flush()
        #print("Heading (ypr):", yaw, pitch, roll)

if __name__ == '__main__':
    main();

