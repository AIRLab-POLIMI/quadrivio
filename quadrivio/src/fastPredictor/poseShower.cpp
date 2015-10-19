#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#define NAME_OF_THIS_NODE "poseShower"

class ROSnode {
private:
  ros::NodeHandle Handle;
  ros::Publisher Publisher;
  unsigned int seq;
public:
  bool Prepare();
  void publishPose();
};

bool ROSnode::Prepare() {
  Publisher = Handle.advertise<geometry_msgs::PoseStamped>("predicted_pose", 20);
  seq  = 0; 
  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
  return true;
}

void ROSnode::publishPose() {
  geometry_msgs::PoseStamped msg;
  tf::StampedTransform pose;  
  listener.lookupTransform("/world", "/base_link", ros::Time::now(), pose);
  msg.header.stamp = pose.stamp_;
  msg.header.seq = seq;
  seq++;
  msg.header.frame_id = "/world";
  tf::quaternionTFToMsg(pose.getRotation(), msg.pose.orientation);
  msg.pose.position.x = pose.getOrigin().getX();
  msg.pose.position.y = pose.getOrigin().getY();
  msg.pose.position.z = pose.getOrigin().getZ();
  Publisher.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  ROSnode node;
  ros::Rate loopRate(25);
  
  if(!node.Prepare())
    return 1;
  
  //check the state periodically
  while(ros::ok()) {
    node.publishPose();
    ros::spinOnce();
    loopRate.sleep();
  }

  return (0);
}
