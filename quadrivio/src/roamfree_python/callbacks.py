#!/usr/bin/env python

import rospy

# --- ROS callback functions to process sensor messages

# handle odometry messages from the encoders
def encoder_odometry(RF, mutex, msg):
  try:
    mutex.acquire()
    
    #t = rospy.get_time() 
    t = msg.header.stamp.secs + float(msg.header.stamp.nsecs)*1e-9
    
    # the encoder message is supposed to contain left and right wheel angular speeds expressed in rad/s
    z = [msg.speed, msg.steer]
 
    # covariance matrix of the noises for the Differential Drive kinematic sensor
    # the top-left 2x2 block is the additive noise on the wheel speed,
    # the remaining parts constraint the DOF which are not fixed by the kinematics,
    # i.e. translate on y, on z, roll and pitch.
    # the values provided are meant to constraint the motion on the xy plane
   
    z_cov = [[1, 0, 0, 0, 0, 0], \
             [0, 1, 0, 0, 0, 0], \
             [0, 0, 1e-6, 0, 0, 0], \
             [0, 0, 0, 1e-6, 0, 0], \
             [0, 0, 0, 0, 1e-6, 0], \
             [0, 0, 0, 0, 0, 1e-6]]  
    
    # feed the RF fusion engine
    RF.addMeasurement('Odo', t, z, z_cov)      
    
  finally:
    mutex.release()

def gps(RF, mutex, msg):
  try:
    mutex.acquire()
    
    #t = rospy.get_time()
    t = msg.header.stamp.secs + float(msg.header.stamp.nsecs)*1e-9

    z = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
 
    z_cov = [[0.7, 0, 0], \
             [0, 0.7, 0], \
             [0, 0, 1]]
                 
    # feed the RF fusion engine
    RF.addMeasurement('GPS', t, z, z_cov)      
    
  finally:
    mutex.release()
    
def magnetometer(RF, mutex, msg):
  try:
    mutex.acquire()
    
    #t = rospy.get_time()
    t = msg.header.stamp.secs + float(msg.header.stamp.nsecs)*1e-9
    
    z = [msg.vector.x, msg.vector.y, msg.vector.z]
  
    z_cov = [[1e-2, 0, 0], \
             [0, 1e-2, 0], \
             [0, 0, 1e-2]]
                 
    # feed the RF fusion engine
    RF.addMeasurement('Mag', t, z, z_cov)      
    
  finally:
    mutex.release()    

def visual_odometry(RF, mutex, msg):
  try:
    mutex.acquire()
    
    #t = rospy.get_time()
    t = msg.header.stamp.secs + float(msg.header.stamp.nsecs)*1e-9

    z_l = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
    z_a = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]

    z_cov_l = [[1e-1, 0, 0], \
               [0, 1e-1, 0], \
               [0, 0, 1e-1]]
          
    z_cov_a = [[1e-1, 0, 0], \
               [0, 1e-1, 0], \
               [0, 0, 1e-2]]
                 
    # feed the RF fusion engine
    RF.addMeasurement('VOlinear', t, z_l, z_cov_l)
    RF.addMeasurement('VOangular', t, z_a, z_cov_a)
    
  finally:
    mutex.release()    
