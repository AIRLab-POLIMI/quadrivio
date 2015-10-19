//This node record and plubish periodically the path
//It works only in autonomous state
#include <ros/ros.h>
#include <heartbeat/HeartbeatClient.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#define NAME_OF_THIS_NODE "pathDisplay"

class ROSnode {
private:
  unsigned long seq;
  ros::NodeHandle Handle;
  ros::Publisher Publisher;
  ros::Publisher PubPoses;
  ros::Subscriber Subscriber;
  nav_msgs::Path currentPath;
  geometry_msgs::PoseArray enuHistory;
  tf::TransformListener listener;
  //state machine client
  HeartbeatClient *hb;  
  void enuCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
public:
  bool Prepare();
  void publishPath();
};

bool ROSnode::Prepare() {
  Publisher = Handle.advertise<nav_msgs::Path>("followed_path", 10);
  PubPoses = Handle.advertise<geometry_msgs::PoseArray>("enu_history", 10);

  Subscriber = Handle.subscribe("enu", 10, &ROSnode::enuCallback, this);

  //Init state machine client
  hb = new HeartbeatClient(Handle);
  hb->start();
  
  currentPath.header.frame_id = "/world";
  enuHistory.header.frame_id = "/world";
  seq = 0;

  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
  return true;
}

void ROSnode::publishPath() {
  if (hb->getState() == heartbeat::State::AUTO) {
    tf::StampedTransform pose;
    listener.lookupTransform("/world", "/base_link", ros::Time(0), pose);
    
    geometry_msgs::PoseStamped element;

    element.header.frame_id = "/world";
    element.pose.position.x = pose.getOrigin().x();
    element.pose.position.y = pose.getOrigin().y();
    element.pose.position.z = pose.getOrigin().z();
    tf::quaternionTFToMsg(pose.getRotation(), element.pose.orientation);
 
    currentPath.poses.push_back(element);
    currentPath.header.seq = seq;
    enuHistory.header.seq = seq;
    seq++;
    currentPath.header.stamp = ros::Time::now();
    enuHistory.header.stamp = ros::Time::now();
    Publisher.publish(currentPath);
    PubPoses.publish(enuHistory);
  } else {
    currentPath.poses.clear();
  }
}

void ROSnode::enuCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  if (hb->getState() == heartbeat::State::AUTO)
    enuHistory.poses.push_back(msg->pose.pose);
  else
    currentPath.poses.clear();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  ROSnode node;
  ros::Rate loopRate(20);
  
  if(!node.Prepare())
    return 1;
  
  //check the state periodically
  while(ros::ok()) {
    node.publishPath();
    ros::spinOnce();
    loopRate.sleep();
  }

  return (0);
}
