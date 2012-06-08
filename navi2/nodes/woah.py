#!/usr/bin/env python
import roslib; roslib.load_manifest('navi2')
import math, rospy, tf

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

import time

rospy.init_node('woahpy')
velPub = rospy.Publisher('output', Twist)
markerPub = rospy.Publisher('debug', MarkerArray)

listener = tf.TransformListener()

maxSpeed       = rospy.get_param('maxSpeed',       2.0)
maxTurnSpeed   = rospy.get_param('maxTurnSpeed',   1.0)
robotWidth     = rospy.get_param('robotWidth',     0.6)
safetyMargin   = rospy.get_param('safetyMargin',   0.35)
extraMargin    = rospy.get_param('extraMargin',    0.15)
turnIntensity  = rospy.get_param('turnIntensity',  1.4)
turnResistance = rospy.get_param('turnResistance', 1.7)
minImpactTime  = rospy.get_param('minImpactTime',  1.0)

deadband = rospy.get_param('minImpactTime',  0.2)

def sign(x):
    return -1.0 if x < 0.0 else 1.0

def d(width, alpha, theta):
    if (theta <= alpha):
        div = (2.0 * math.sin(alpha - theta))
        if (div == 0.0):
            return float('+inf')
        else:
            return (width * math.sin(math.pi / 2.0 - alpha)) / div
    else:
        return d(width, -alpha, -theta)

def minDist(width, alphaLeft, alphaRight, ranges):
    def corridorDist((theta, scan)):
        if (theta < alphaRight):
            if (scan < d(width, alphaRight, theta)):
                return scan
            else:
                return float('+inf')
        elif (theta > alphaRight and theta < alphaLeft):
            return scan
        else: #(theta > alphaLeft)
            if (scan < d(width, alphaLeft, theta)):
                return scan
            else:
                return float('+inf')

    return min(map(corridorDist, ranges))

def forwardSpeed(width, alpha, goalDist, goalFlag, ranges):
    global safetyMargin, minImpactTime
    
    alphaLeft = max(0.0, alpha)
    alphaRight = min(0.0, alpha)
    safeDist = minDist(width, alphaLeft, alphaRight, ranges) - safetyMargin * 2.0
    distance = min(goalDist, safeDist) if goalFlag else safeDist

    showDebug(alphaLeft, alphaRight, distance)
    
    return min(maxSpeed, distance / minImpactTime)

def turnSpeed(alpha):
    global maxTurnSpeed, turnIntensity
    
    ret = sign(alpha) * maxTurnSpeed
    return ret * (2.0 * abs(alpha / math.pi) ** (1.0 / turnIntensity))

def goalProgress(width, goalAlpha, goalDist, alpha, ranges):
    global extraMargin, turnResistance

    x = min(goalDist, minDist(width + extraMargin, alpha, alpha, ranges))
    return x * (math.cos(abs(goalAlpha - alpha)) ** (turnIntensity + 0.0j)).real

def findDirection(width, goalAlpha, goalDist, ranges):
    def findBetterAlpha((progAlpha, prog), alpha):
        newProg = goalProgress(width, goalAlpha, goalDist, alpha, ranges)
        if (newProg > prog):
            return (alpha, newProg)
        else:
            return (progAlpha, prog)

    angles = [p[0] for p in ranges]
    return reduce(findBetterAlpha, angles, (0.0, float("-inf")))[0]

def woahForward(goalAlpha, goalDist, goalFlag, ranges):
    global robotWidth, safetyMargin

    width = robotWidth + safetyMargin
    alpha = findDirection(width, goalAlpha, goalDist, ranges)
    speed = forwardSpeed(width, alpha, goalDist, goalFlag, ranges)
    turn = turnSpeed(alpha)

    return (speed, turn)

def woahBackward(goalAlpha):
    global maxTurnSpeed
    
    return (0.0, sign(goalAlpha) * maxTurnSpeed)

def woah(goalAlpha, goalDist, goalFlag, ranges, senseBounds):
    if (goalAlpha >= senseBounds[0] and goalAlpha <= senseBounds[1]):
        return woahForward(goalAlpha, goalDist, goalFlag, ranges)
    else:
        return woahBackward(goalAlpha)

lastPath = Path()
modPath = Path()
lastPose = [0,0,0]
        
def showDebug(alphaLeft, alphaRight, distance):
    global lastPose, modPath
    markers = MarkerArray()
    
    ###############################
    mark = Marker()
    mark.header.stamp = rospy.get_rostime()
    mark.header.frame_id = '/map'
    mark.action = 0
    mark.ns = 'WOAH'
    mark.id = 0
    mark.type = 5
    mark.lifetime = rospy.Duration(0)
    mark.scale.x = 0.1
    mark.pose.position.x = lastPose[0]
    mark.pose.position.y = lastPose[1]
    mark.pose.orientation.x = quaternion_from_euler(0,0,lastPose[2])[0]
    mark.pose.orientation.y = quaternion_from_euler(0,0,lastPose[2])[1]
    mark.pose.orientation.z = quaternion_from_euler(0,0,lastPose[2])[2]
    mark.pose.orientation.w = quaternion_from_euler(0,0,lastPose[2])[3]
    
    mark.color.b = 1.0
    mark.color.a = 1.0
    
    mark.colors.append(ColorRGBA())
    mark.colors[0].b = 1.0
    mark.colors[0].a = 1.0
    mark.colors.append(mark.colors[-1])
    mark.colors.append(mark.colors[-1])
    mark.colors.append(mark.colors[-1])
    
    mark.points.append(Point())
    mark.points[-1].y = safetyMargin
    mark.points.append(Point())
    mark.points[-1].x = math.cos(alphaLeft) * distance
    mark.points[-1].y = math.sin(alphaLeft) * distance + safetyMargin
    
    mark.points.append(Point())
    mark.points[-1].y = -safetyMargin
    mark.points.append(Point())
    mark.points[-1].x = math.cos(alphaRight) * distance
    mark.points[-1].y = math.sin(alphaRight) * distance - safetyMargin
    
    markers.markers.append(mark)
    #################################
    mark = Marker()
    mark.header.stamp = rospy.get_rostime()
    mark.header.frame_id = '/map'
    mark.action = 0
    mark.ns = 'WOAH'
    mark.id = 1
    mark.type = 5
    mark.lifetime = rospy.Duration(0)
    mark.scale.x = 0.1
    mark.pose.orientation.x = 0.0
    mark.pose.orientation.y = 0.0
    mark.pose.orientation.z = 0.0
    mark.pose.orientation.w = 1.0
    
    mark.color.b = 1.0
    mark.color.a = 1.0
    
    mark.colors.append(ColorRGBA())
    mark.colors[-1].r = 1.0
    mark.colors[-1].a = 1.0
    mark.colors.append(mark.colors[-1])
    
    mark.points.append(Point())
    mark.points[-1].x = lastPose[0]
    mark.points[-1].y = lastPose[1]
    mark.points.append(Point())
    mark.points[-1].x = modPath.poses[-1].pose.position.x
    mark.points[-1].y = modPath.poses[-1].pose.position.y
    
    markers.markers.append(mark)
    #################################
    
    markerPub.publish(markers)
        
def gotPath(msg):
    global lastPath

    lastPath = msg

def gotSense(msg):
    global lastPath, lastPose, modPath
    ##
    clock = time.time()
    ##
    modPath = lastPath
    
    if (len(lastPath.poses) == 0):
        velPub.publish(Twist())
        print 'No Path'
        return
    
    ranges = []
    angMin = msg.angle_min
    angMax = msg.angle_max
    startAngle = msg.angle_min

    for i in xrange(len(msg.ranges)):
        pair = (ang, r) = (msg.angle_min + float(i) * msg.angle_increment, msg.ranges[i])
        
        if (r < msg.range_min or r > msg.range_max or i % 2 == 1):
            pass
        else:
            ranges.append(pair)
    
    ##
    print 'Time pre-tf: ' + str(time.time() - clock)
    ##
    
    try:
        listener.waitForTransform("/map", msg.header.frame_id, msg.header.stamp, rospy.Duration(1.0))
        (trans,rot) = listener.lookupTransform('/map', msg.header.frame_id, msg.header.stamp)
    except (tf.LookupException, tf.ConnectivityException):
        return
    
    ##
    print 'Time post-tf: ' + str(time.time() - clock)
    ##
    
    yaw = euler_from_quaternion(rot)[2]
    
    lastPose = [trans[0], trans[1], yaw]
    
    goalAlpha = math.atan2(modPath.poses[-1].pose.position.y - trans[1],
                           modPath.poses[-1].pose.position.x - trans[0]) - yaw
                           
    goalDist = math.sqrt((trans[1] - modPath.poses[-1].pose.position.y) ** 2.0 +
                         (trans[0] - modPath.poses[-1].pose.position.x) ** 2.0)

    cmd = Twist()

    #print ('Pose', lastPose)
    #print ('Target', lastTar)
    
    if goalAlpha > math.pi:
        goalAlpha -= math.pi * 2.0
    elif goalAlpha < -math.pi:
        goalAlpha += math.pi * 2.0
    
    ##
    print 'Time pre-woah: ' + str(time.time() - clock)
    ##
    
    if (goalDist > 0.5):
        (cmd.linear.x, cmd.angular.z) = woah(goalAlpha, goalDist, len(modPath.poses) == 1, ranges, [angMin, angMax])
        
        #if (cmd.linear.x < deadband):
        #    cmd.angular.z = sign(goalAlpha) * maxTurnSpeed
        
        print ((goalAlpha, goalDist), cmd.linear.x, cmd.angular.z)
        #(cmd.linear.x, cmd.angular.z) = (0.0, 0.0)
    else:
        (cmd.linear.x, cmd.angular.z) = (0.0, 0.0)
        print "Arrived"
    
    ##
    print 'Time post-woah: ' + str(time.time() - clock)
    ##
    
    velPub.publish(cmd)

rospy.Subscriber('inputPath', Path, gotPath, queue_size=1)
rospy.Subscriber('inputLaser', LaserScan, gotSense, queue_size=1)

rospy.spin()

