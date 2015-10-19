#!/usr/bin/env python

#
# this node subscribes to the /enu and /magnetic topics
#
# as soon as a sensor reading is received on both, an initial guess
# of the robot pose and orientation is computed and the parameter
# /roamfree/initialPose is set accordingly in the parameter server
#
# once done, the node terminates.
#

from sys import exit
import rospy

from threading import Lock

from math import sin, cos, atan2, pi

from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PoseWithCovarianceStamped

ZMag = None
ZGPS = None

mutex = Lock()

def gps_cb(msg):
  global mutex
  global ZGPS
  
  mutex.acquire()
  ZGPS = msg
  mutex.release()
  
  
def mag_cb(msg):
  global mutex
  global ZMag
  
  mutex.acquire()
  ZMag = msg  
  mutex.release()
 
rospy.init_node('firstPose', anonymous=True)


# load needed parameters
try:
  gpsSO = rospy.get_param('/roamfree/gpsDisplacement')
    
except KeyError as e:
  rospy.logfatal('\'%s\'not found in Parameter Server', e.args[0])
  exit(1)

# subscribe to topics

rospy.Subscriber("/enu", PoseWithCovarianceStamped, gps_cb)
rospy.Subscriber("/magnetic", Vector3Stamped, mag_cb)

# loop at 10 Hz till both GPS and Mag measurements have been received

r = rospy.Rate(10)
done = False

while not rospy.is_shutdown() and not done:    

  mutex.acquire()  
  if ZMag != None and ZGPS != None:
    
    # I have both readings, set the /roamfree/initialPose parameter
    
    theta = atan2(ZMag.vector.x, ZMag.vector.y)
    Q = {'w': cos(theta/2.0), 'x': 0.0, 'y': 0.0, 'z': sin(theta/2.0)}
    POSE = {
      'x': ZGPS.pose.pose.position.x - gpsSO['x']*cos(theta) + gpsSO['y']*sin(theta), \
      'y': ZGPS.pose.pose.position.y - gpsSO['y']*cos(theta) - gpsSO['x']*sin(theta), \
      'z': -gpsSO['z'] + ZGPS.pose.pose.position.z \
    }
  
    rospy.set_param('/roamfree/initialPose/position', POSE)
    rospy.set_param('/roamfree/initialPose/orientation', Q)    
    
    done = True
  
  mutex.release()

  r.sleep()
