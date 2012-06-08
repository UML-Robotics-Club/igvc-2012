#!/usr/bin/env python
import roslib; roslib.load_manifest('navi2')
import math, rospy, tf

from nav_msgs.msg import Path

rospy.init_node('pathFilter')
pathPub = rospy.Publisher('output', Path)

listener = tf.TransformListener()

lastPath = Path()
newPath = False

def gotPath(msg):
    global lastPath, newPath
    newPath = True
    lastPath = msg
    
    print 'GotPath'
    
rospy.Subscriber('input', Path, gotPath, queue_size=1)

def main():
    global lastPath, newPath
    
    modPath = Path()
    
    rate = rospy.Rate(10)
    crumbs = []
    
    while not rospy.is_shutdown():
        rate.sleep()
        
        if newPath:
            print 'See new Path'
            modPath = lastPath
            modPath = reduce(clearCrumb, crumbs, modPath)
            newPath = False
            crumbs = []
        
        try:
            now = rospy.get_rostime()
            listener.waitForTransform("/map","/base_link", now, rospy.Duration(1.0))
            (trans,rot) = listener.lookupTransform('/map', 'base_link', now,)
        except:
            continue
        
        crumbs.append(trans)
        
        if (len(modPath.poses) > 1):
            print 'Checking Path'
            modPath = clearCrumb(modPath, crumb[-1])
            
            pathPub.publish(modPath)
            
        
            
def clearCrumb(path, crumb):
        
    if math.sqrt((path.poses[-1].pose.position.x - crumb[0]) ** 2.0 + 
                 (path.poses[-1].pose.position.y - crumb[1]) ** 2.0) < 0.8:
        path.poses = path.poses[0:-1]
        
    return path
    
main()