#include "ros/ros.h"
#include <quadrivio_msgs/SetPoint.h>
#include <iostream>
#include <fstream>

#define NAME_OF_THIS_NODE "simulatedInput"

class ROSnode {
private:
  ros::NodeHandle Handle;
public:
  ros::Publisher Publisher;  
  bool Prepare();
};

bool ROSnode::Prepare() {
  //Init publisher and subscriber
  Publisher = Handle.advertise<quadrivio_msgs::SetPoint>("setpoint", 20);
  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
  return true;
}

int main(int argc, char **argv) {
  using namespace std;
  ros::init(argc, argv, NAME_OF_THIS_NODE);

  ROSnode node;
  ros::Rate loopRate(20);
  
  if(!node.Prepare())
    return 1;  
  
  ifstream inputs;
  inputs.open("/home/quadrivio/inputs");
  if(inputs.is_open())
    ROS_INFO("file aperto");
  else
    return 1;
	
  while(ros::ok()){
    string str;
    inputs >> str;
    if(!inputs.eof()) {
      quadrivio_msgs::SetPoint msg;
      ROS_INFO("%s", str.c_str());
      msg.speed = atof(str.c_str());
      msg.steer = 0;
      msg.brake = 0;
      msg.source = quadrivio_msgs::SetPoint::JOYPAD;
      node.Publisher.publish(msg);
    }
    else
      return 1;
    ros::spinOnce();
    loopRate.sleep();
  }
  
  return (0);
}
