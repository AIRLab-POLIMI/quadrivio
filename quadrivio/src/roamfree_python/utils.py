import rospy
from geometry_msgs.msg import Pose

def getPoseFromParameterServer(name):  
  try:
    param = rospy.get_param(name)    
    return Pose(param['position'], param['orientation'])
  except TypeError:
    rospy.logfatal('Bad parameter structure for %s', name)
    
  return None
  
def poseMsgToList(p):
  return [p.position.x, p.position.y, p.position.z, p.orientation.w, p.orientation.x. p.orientation.y, p.orientation.z]
  
def rosTimeFromDouble(t):
  ht = rospy.Time()
  
  ht.secs = int(t)
  ht.nsecs = int((t-int(t))*1e9)
  
  return ht 
  
