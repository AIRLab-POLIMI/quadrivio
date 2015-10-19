//This node simulates a GPS
//Topic published: /enu
//Message: geometry_msgs/PoseWithCovarianceStamped
//Topic subscribed: /vrep/simuGPS
//Message: geometry_msgs/Point32
//Parameters: threshold - probability of a downtime (0 - 100)
//            duration  - duration of every downtime in second
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

#define NAME_OF_THIS_NODE "realPose"

class ROSnode {
private:
  ros::NodeHandle Handle;
  ros::Subscriber Subscriber;
  ros::Publisher Publisher;
  void realPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  
public:
  bool Prepare();
};

bool ROSnode::Prepare() {
  
  //init subscriber and publisher
  Subscriber = Handle.subscribe("vrep/realPose", 10, &ROSnode::realPoseCallback, this);
  Publisher = Handle.advertise<geometry_msgs::PoseStamped>("vrep/realPoseFull", 20);
  
  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
  
  return true;
}

void ROSnode::realPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  //read the message and publish it as a PoseStamped message
  geometry_msgs::PoseStamped nmsg;
  
  nmsg.header = msg->header;
  nmsg.pose = msg->pose;
  nmsg.header.frame_id = "/world";
  nmsg.pose.position.z = msg->pose.position.z + 0.3;
  
  Publisher.publish(nmsg);  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  ROSnode node;
  
  if(!node.Prepare())
    return 1;
  
  ros::spin(); 
  
  return (0);
}
