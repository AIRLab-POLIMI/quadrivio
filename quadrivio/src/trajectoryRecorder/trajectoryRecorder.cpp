//This node record a trajectory and publish it on a topic
//Command are isseud via joypad

//Topics published:
// /path           - Message: quadrivio_msgs/PathWithVelocity
//The complete recoreded path
// /visualize_path - Message: nav_msgs/Path
//The partial path published periodically
// /trjRec_state   - Message: std_msgs/String
//The current state of the internal state machine

//Topics subscribed:
// /roamfree/odometry - nav_msgs/Odometry
// /joy               - sensor_msgs/Joy

//Parameter: dead_zone - minimum distance between two pose

#include <ros/ros.h>
#include <ros/timer.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <quadrivio_msgs/PathWithVelocity.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#define NODE_NAME "trajectoryRecorder"

//Internal state of the node
enum State {
  WAIT, RECORD, TRAJECTORY
};

class ROSnode {
private:
  unsigned long seqPath;
  unsigned long seqVPath;
  unsigned long seqPose;

  State curState;

  double deadZone;

  ros::NodeHandle Handle;

  //ros::Subscriber subPose;
  ros::Subscriber subJoypad;
  
  tf::TransformListener listener;

  ros::Publisher pubPath;
  ros::Publisher pubVPath;
  ros::Publisher pubState;

  ros::Timer timer;

  nav_msgs::Path currentPath;
  std::vector<double> velocity;

  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  void timeoutCallback(const ros::TimerEvent& event);

public:
  bool Prepare();
  void pathCreator();
};

bool ROSnode::Prepare() {
  seqPath = 0;
  seqVPath = 0;
  seqPose = 0;
  curState = WAIT;

  //Retrieve parameter
  if (Handle.getParam("/trajectoryRecorder/dead_zone", deadZone)) {
    ROS_INFO("Node %s: retrieved parameter dead_zone.", ros::this_node::getName().c_str());
  } else {
    ROS_FATAL("Node %s: unable to retrieve parameter dead_zone.", ros::this_node::getName().c_str());
    return false;
  }

  deadZone = deadZone * deadZone;

  //Init subscribers
  //subPose = Handle.subscribe("/RR/pose", 10, &ROSnode::poseCallback, this);
  subJoypad = Handle.subscribe("joy", 10, &ROSnode::joyCallback, this);

  //Init publishers
  pubPath = Handle.advertise<quadrivio_msgs::PathWithVelocity>("path", 10);
  pubVPath = Handle.advertise<nav_msgs::Path>("visualize_path", 10);
  pubState = Handle.advertise<std_msgs::String>("trjRec_state", 10);

  //Init timers
  timer = Handle.createTimer(ros::Duration(0.5), &ROSnode::timeoutCallback, this, false);

  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());

  currentPath.header.frame_id = "/world";
}

void ROSnode::pathCreator() {
  if (curState == RECORD) {
    tf::StampedTransform transform;
    listener.lookupTransform("/world", "/base_link", ros::Time(0), transform);
    geometry_msgs::PoseStamped element;
    
    element.header.seq = seqPose;
    element.header.stamp = transform.stamp_;
    element.header.frame_id = "/world";
    element.pose.position.x = transform.getOrigin().x();
    element.pose.position.y = transform.getOrigin().y();
    element.pose.position.z = transform.getOrigin().z();
    tf::quaternionTFToMsg(transform.getRotation(), element.pose.orientation);

    // TODO: get linear velocity setpoint somewhere

    if (currentPath.poses.size() > 0) {
      geometry_msgs::PoseStamped back = currentPath.poses.back();

      //Check the distance between last pose and new pose
      double dist = pow(back.pose.position.x - element.pose.position.x, 2)
          + pow(back.pose.position.y - element.pose.position.y, 2)
          + pow(back.pose.position.z - element.pose.position.z, 2);

      if (dist > deadZone) {
	//Save if distant enough
        currentPath.poses.push_back(element);
        velocity.push_back(2.0);
      }
    } else {
      //There are no elements, save it anyway
      currentPath.poses.push_back(element);
      velocity.push_back(2.0);
    }
  }
}

void ROSnode::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  //Use joypad input to control the internal state machine
  std_msgs::String str;
  std::stringstream ss;
  switch (curState) {
  case WAIT:
    //button A
    if (msg->buttons[0] == 1) { // go to the recording state
      curState = RECORD;
      ss << "RECORD";
      str.data = ss.str();
      pubState.publish(str);
      ROS_INFO("Current State: RECORD");
    }
    break;
  case RECORD:
    //button A
    if (msg->buttons[0] == 1) { // go to the trajectory state
      curState = TRAJECTORY;
      ss << "TRAJECTORY";
      str.data = ss.str();
      pubState.publish(str);
      ROS_INFO("Current State: TRAJECTORY");
    }
    break;
  case TRAJECTORY:
    //button Y
    if (msg->buttons[3] == 1) { // publish trajectory to follow
      quadrivio_msgs::PathWithVelocity path;
      path.header.seq = seqPath;
      seqPath++;
      path.header.stamp = ros::Time::now();
      path.header.frame_id = "/world";
      path.path = currentPath;
      path.velocities = velocity;

      pubPath.publish(path);
    }

    //button X
    if (msg->buttons[2] == 1) { // delete trajectory and go to the wait state
      curState = WAIT;
      currentPath.poses.clear();
      seqPose = 0;
      ss << "WAIT";
      str.data = ss.str();
      pubState.publish(str);
      ROS_INFO("Current State: WAIT");
    }
    break;
  }
}

//Every timeout publish a partial path
void ROSnode::timeoutCallback(const ros::TimerEvent& event) {
  if (curState == RECORD || curState == TRAJECTORY) {
    quadrivio_msgs::PathWithVelocity path;
    path.header.seq = seqVPath;
    seqVPath++;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "/world";
    path.path = currentPath;
    path.velocities = velocity;
    pubVPath.publish(path.path);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NODE_NAME);

  ROSnode mNode;

  mNode.Prepare();
  
  ros::Rate loopRate(20);
  
  while (ros::ok()) {
    mNode.pathCreator();
    ros::spinOnce();
    loopRate.sleep();
  }
  
  return (0);
}
