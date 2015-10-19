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
import tf

from threading import Lock

from math import sin, cos, atan2, pi

from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import PoseWithCovarianceStamped

ZMag = None
ZGPS = None

mutex = Lock()

def gps_cb(msg):
  global mutex
  global ZGPS
  
  mutex.acquire()
  ZGPS = msg
  ts = msg.header.stamp
  mutex.release()
  
  
def mag_cb(msg):
  global mutex
  global ZMag
  
  mutex.acquire()
  ZMag = msg  
  mutex.release()
 
rospy.init_node('firstPose', anonymous=True)


# wait for the transformation from /roamfree to /gps is available

r = rospy.Rate(10.0)
tf_listener = tf.TransformListener()

while not rospy.is_shutdown():
  try:
    (GPS_trans,GPS_rot) = tf_listener.lookupTransform('/roamfree', '/gps', rospy.Time(0))
    break

  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    pass

  r.sleep()
  continue

# subscribe to topics

rospy.Subscriber('/enu', PoseWithCovarianceStamped, gps_cb)
rospy.Subscriber('/magnetic', MagneticField, mag_cb)

pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, latch = True)

# loop at 10 Hz till both GPS and Mag measurements have been received

r = rospy.Rate(1)

while not rospy.is_shutdown():

  mutex.acquire()  
  if ZMag != None and ZGPS != None:
    
    # I have both readings, set the /roamfree/initialPose parameter
    
    theta = atan2(ZMag.magnetic_field.x, ZMag.magnetic_field.y)
    
    msg = PoseWithCovarianceStamped()

    msg.header.stamp = rospy.Time.now()

    msg.pose.pose.position.x = ZGPS.pose.pose.position.x - GPS_trans[0]*cos(theta) + GPS_trans[1]*sin(theta)
    msg.pose.pose.position.y = ZGPS.pose.pose.position.y - GPS_trans[1]*cos(theta) - GPS_trans[0]*sin(theta)
    msg.pose.pose.position.z = -GPS_trans[2] + ZGPS.pose.pose.position.z

    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = sin(theta/2.0)
    msg.pose.pose.orientation.w = cos(theta/2.0)
    
    pub.publish(msg)    
      
  mutex.release()

  r.sleep()
