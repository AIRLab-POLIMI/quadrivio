#!/usr/bin/env python
import rospy
from roamros_msgs.msg import SingleTrackAckermannOdometryStamped

def callback(msg):    
    callback.offset = ((rospy.Time.now()-msg.header.stamp).to_sec() + callback.offset*callback.N)/(callback.N+1)
    callback.N = callback.N + 1

    rospy.loginfo("ts: {1}, average delay: {0}".format(callback.offset, msg.header.stamp.to_sec()));
    
def listener():

    callback.offset = 0;
    callback.N = 1;

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('odometry_monitor', anonymous=True)

    rospy.Subscriber("/PLC/ackermann_odometry", SingleTrackAckermannOdometryStamped, callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()