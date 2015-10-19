#!/usr/bin/env python

# 
# This node sends a simple, hardcoded path
# 
# It is supposed to be used to test the Trajectory Control in simulation
#

import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

rospy.init_node('sendSimplePath', anonymous=True)

pub = rospy.Publisher("/path", Path)

rospy.sleep(1)

msg = Path()

msg.header.frame_id = '/world'

x = PoseStamped()
x.pose.position.x = -3
x.pose.position.y = -3
msg.poses.append(x)

x = PoseStamped()
x.pose.position.x = -3
x.pose.position.y = 3
msg.poses.append(x)

x = PoseStamped()
x.pose.position.x = 3
x.pose.position.y = 3
msg.poses.append(x)

x = PoseStamped()
x.pose.position.x = 3
x.pose.position.y = -3
msg.poses.append(x)

x = PoseStamped()
x.pose.position.x = -3
x.pose.position.y = -3
msg.poses.append(x)

pub.publish(msg)
