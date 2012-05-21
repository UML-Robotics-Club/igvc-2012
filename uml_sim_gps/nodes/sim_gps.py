#!/usr/bin/env python
import roslib; roslib.load_manifest('uml_sim_gps')
import rospy

CONFIG = {
    'UPDATE_DELAY': 0.0,
    'BASE_LAT'    :  42.656516,
    'BASE_LON'    : -71.323866,
    'UTM_ZONE'    : '19T',
    'SIM_ERROR'   : 1.0,
    'ERR_PERIOD'  : 120,
}

import sys
import gps_stage
import gps_gazebo
import tf

from sensor_msgs.msg   import NavSatFix
from geometry_msgs.msg import Pose2D

from pyproj import Proj
from time import time
from math import cos

conv = Proj(proj='utm',zone='19T',ellps='WGS84')
(base_utm_e, base_utm_n) = conv(CONFIG['BASE_LON'], CONFIG['BASE_LAT'])
(base_xx, base_yy) = (50.0, 50.0)

rospy.init_node('sim_gps')
gps = rospy.Publisher('robot/gps', NavSatFix)
pos = rospy.Publisher('robot/pose', Pose2D)
tfBr = tf.TransformBroadcaster()

rospy.loginfo("BASE UTM: " + str(base_utm_e) + ' ' + str(base_utm_n))

EP = CONFIG['ERR_PERIOD']

def got_position(xx, yy, aa, stamp):
    global EP

    fix = NavSatFix()
    fix.header.stamp = stamp
    fix.header.frame_id = "sim_gps"
    fix.status.status = 0
    fix.status.service = 1

    utm_e = base_xx + base_utm_e + xx + CONFIG['SIM_ERROR']*cos((yy+time()) / EP)
    utm_n = base_yy + base_utm_n + yy + CONFIG['SIM_ERROR']*cos((utm_e / 3.14) / EP)
    (lon, lat) = conv(utm_e, utm_n, inverse=True)

    fix.latitude  = lat
    fix.longitude = lon
    fix.altitude  = 1.3

    #print "got_position: %f %f" % (lat, lon)

    gps.publish(fix)
    pos.publish(Pose2D(utm_e, utm_n, aa))
    tfBr.sendTransform((utm_e - base_xx, utm_n - base_yy, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "/map", "/gps")

if sys.argv[1] == '-s':
    gps_stage.sim_gps(CONFIG, got_position)
elif sys.argv[1] == '-g':
    gps_gazebo.sim_gps(CONFIG, got_position)
else:
    print "Usage:"
    print "  rosrun uml_sim_gps sim_gps.py <mode>"
    print "  <mode> is -g for gazebo, -s for stage."
