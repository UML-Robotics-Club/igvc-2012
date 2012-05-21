#!/usr/bin/env python
import roslib; roslib.load_manifest('os5000')
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
        yaw   = float(msg[1]);
        pitch = float(msg[2]);
        roll  = float(msg[3]);
        #out = quaternion_from_euler(roll,pitch,yaw)#,'rxyz')
        out = quaternion_from_euler(0,0,yaw)#,'rxyz')

	msg = QuaternionStamped()
	msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        msg.quaternion.x = out[0]
        msg.quaternion.y = out[1]
        msg.quaternion.z = out[2]
        msg.quaternion.w = out[3]

        pub.publish(msg)

        #print("Heading (ypr):", yaw, pitch, roll)

if __name__ == '__main__':
    main();

