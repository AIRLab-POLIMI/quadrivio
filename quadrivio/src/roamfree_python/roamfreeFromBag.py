#!/usr/bin/env python

import os
import sys
from threading import Lock
from functools import partial

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PoseStamped
from quadrivio.msg import QuadOdometry

import callbacks
import utils

from time import sleep

# the ROAMFREE library path, so that python can locate the RF module
sys.path.append('/home/davide/Code/ROAMFREE_build/Debug/lib')
sys.path.append('/home/davide/Code/roamfree/trunk/ROAMROS/ROAMROSutils')

from libPyROAMFREE import *
from ROAMpythonUtils import *
from bagPlayer import BagPlayer

def listener():
  
  # clean roamfree core log folder  
  os.popen("rm -rf /tmp/roamfree")
  os.popen("mkdir /tmp/roamfree")
    
  # init ROS node
  rospy.init_node('roamfree', anonymous=True)

  # load parameters form parameter server
  
  try:
    poseWindowLenght = rospy.get_param('/roamfree/poseWindowLength')
    ackermannParams = rospy.get_param('/roamfree/ackermannParameters')
    frequency = rospy.get_param('/roamfree/frequency')
    gpsSO = rospy.get_param('/roamfree/gpsDisplacement')
    EarthH = rospy.get_param('/roamfree/EarthMagneticField')
    VOSO = rospy.get_param('/roamfree/stereoDisplacement')
    VOqOS = rospy.get_param('/roamfree/stereoMisalignment')
    
  except KeyError as e:
    rospy.logfatal('\'%s\'not found in Parameter Server', e.args[0])
    sys.exit(1)
      
  # init the bag player
    
  bag = BagPlayer(sys.argv[1], startAt=float(sys.argv[2]))
  
  # wait until the /roamfree/initialPose parameter becomes available
  
  iP = dict()
  while any(iP) == False:
    try:
      iP['position'] = rospy.get_param('/roamfree/initialPose/position')
      iP['orientation'] = rospy.get_param('/roamfree/initialPose/orientation')
    except KeyError:
      iP.clear()
      
      bag.play(1)
      
      
    if rospy.is_shutdown(): #prevent being stuck here
      sys.exit(1)        
  
  # initial pose lodead from parameter server
  x0 = [ iP['position']['x'], \
         iP['position']['y'], \
         iP['position']['z'], \
         iP['orientation']['w'], \
         iP['orientation']['x'], \
         iP['orientation']['y'], \
         iP['orientation']['z'] \
       ]
  
  # mutex for concurrent access to the RF fusion engine
  mutex = Lock()
  
  # --- spawn the RF fusion engine  
  RF = g2oFilter(poseWindowLenght, rospy.get_time(), x0, SolverMethod.GaussNewton)

  # --- configure logical sensors
  
  # -------------------- LOGICAL SENSORS -------------------------------

  # Ackermann kinematic logical sensor, called 'Odo' and being the master sensor
  RF.addSensor('Odo', MeasurementTypes.AckermannOdometer, True)

  # 'Odo' parameters, everyone is Fixed (i.e. last parameter set to True)
  # wheel radius and distance, roto-translation wrt odometric reference frame, everything

  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Odo_kSpeed', [ackermannParams['kSpeed']], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Odo_kSteer', [ackermannParams['kSteer']], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Odo_psiSteer', [ackermannParams['psiSteer']], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Odo_L', [ackermannParams['L']], True)

  # it is placed AT the odometric reference frame
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Odo_SOx', [0.0], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Odo_SOy', [0.0], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Odo_SOz', [0.0], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Odo_qOSx', [0.0], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Odo_qOSy', [0.0], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Odo_qOSz', [0.0], True)


  # Absolute Position logical sensor, called 'GPS'
  RF.addSensor('GPS', MeasurementTypes.AbsolutePosition, False)

  # it is placed AT the odometric reference frame
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'GPS_SOx', [gpsSO['x']], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'GPS_SOy', [gpsSO['y']], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'GPS_SOz', [gpsSO['z']], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'GPS_qOSx', [0.0], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'GPS_qOSy', [0.0], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'GPS_qOSz', [0.0], True)


  # Magnetic Field logical sensor
  RF.addSensor('Mag', MeasurementTypes.VectorField, False)

  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Mag_SOx', [0.0], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Mag_SOy', [0.0], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Mag_SOz', [0.0], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Mag_qOSx', [0.0], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Mag_qOSy', [0.0], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'Mag_qOSz', [0.0], True)

  RF.addConstantParameter(ParameterTypes.Matrix3D, 'Mag_R', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], True)
  RF.addConstantParameter(ParameterTypes.Euclidean3D, 'Mag_S', [0.0, 0.0, 0.0], True)
  RF.addConstantParameter(ParameterTypes.Euclidean3D, 'Mag_h', [EarthH['x'], EarthH['y'], EarthH['z']], True)


  # Visual Odometry logical sensor

  RF.addSensor('VOlinear', MeasurementTypes.LinearVelocity, False)

  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'VOlinear_SOx', [VOSO['x']], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'VOlinear_SOy', [VOSO['y']], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'VOlinear_SOz', [VOSO['z']], True)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'VOlinear_qOSx', [VOqOS['qx']], False)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'VOlinear_qOSy', [VOqOS['qy']], False)
  RF.addConstantParameter(ParameterTypes.Euclidean1D, 'VOlinear_qOSz', [VOqOS['qz']], True)

  RF.addSensor('VOangular', MeasurementTypes.AngularVelocity, False)
  shareDefaultParameters(RF, 'VOlinear','VOangular')

  RF.addConstantParameter(ParameterTypes.Euclidean3D, 'VOangular_G', [1, 1, 1], True)
  RF.addConstantParameter(ParameterTypes.Euclidean3D, 'VOangular_B', [0, 0, 0], True)

  # --------------------------------------------------------------------
  
  # --- do ROS related stuff
    
  # instantiate python partial functions to pass parameters to callbacks
  
  enc_cb = partial(callbacks.encoder_odometry, RF, mutex)
  gps_cb = partial(callbacks.gps, RF, mutex)
  mag_cb = partial(callbacks.magnetometer, RF, mutex)
  vo_cb = partial(callbacks.visual_odometry, RF, mutex)

  # subscribe to sensor topics
  rospy.Subscriber("PLC/odometry", QuadOdometry, enc_cb)
  rospy.Subscriber("/enu", PoseStamped, gps_cb)
  rospy.Subscriber("/magnetic", Vector3Stamped, mag_cb)
  rospy.Subscriber("/stereo_odometer/odometry", Odometry, vo_cb)
  
  # --- main loop, accumulate 3 min of readings
  for i in range(18):
    
    bag.play(10)
    sleep(10)
    
    RF.offlineEstimate(50)
  
  # --- unfix relevant calibration parameters
  
  RF.setParameterFixed('VOlinear_qOSx', False)
  RF.setParameterFixed('VOlinear_qOSx', False)
  RF.setParameterFixed('VOlinear_SOx', False)
  
  RF.offlineEstimate(100)  
    
    
    #raw_input()
      
if __name__ == '__main__':
    try:
      listener()
    except rospy.ROSInterruptException:
      pass  
