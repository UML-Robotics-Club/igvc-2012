#!/usr/bin/env python
import roslib; roslib.load_manifest('uml_sim_gps')
import rospy

from gazebo.srv import GetModelState
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion

import time
import sys

MODEL_NAME="robot"

callback = None

def publish_position(state):
    quat = state.pose.orientation
    (_, _, angle) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    posi = state.pose.position
    callback(posi.x, posi.y, angle, rospy.Time.now())
    
def sim_gps(CONFIG, cb):
    global callback
    callback = cb

    rospy.wait_for_service('gazebo/get_model_state')
    get_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

    while True:
        try:
            state = get_state(MODEL_NAME, "")
            publish_position(state)
            rospy.sleep(CONFIG['UPDATE_DELAY'])
        except rospy.ServiceException, e:
            print "Service did not process request: %s"%str(e)
            sys.exit(1)
        except rospy.exceptions.ROSInterruptException, e:
            print "Shutting down"
            sys.exit(0)
