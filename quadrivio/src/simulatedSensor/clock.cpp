#include "ros/ros.h"
#include <vrep_common/VrepInfo.h>
#include <vrep_common/simRosGetInfo.h>
#include <rosgraph_msgs/Clock.h>
#include <chrono>
#include <thread>

#define NODE_NAME "clock"

class ROSnode {
private:
	ros::NodeHandle Handle;
	ros::Publisher publisher;
  ros::ServiceClient service;
  rosgraph_msgs::Clock clock;
public:
	bool Prepare();
  void publishClock();
};

bool ROSnode::Prepare() {
	//Init subscribers
  publisher = Handle.advertise<rosgraph_msgs::Clock>("clock", 20);
  service = Handle.serviceClient<vrep_common::simRosGetInfo>("vrep/simRosGetInfo");

  vrep_common::simRosGetInfo info;
  
  if(service.call(info)) {
    ros::Time simTime(info.response.simulationTime);
    clock.clock = simTime;
  }

	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
	return true;
}

void ROSnode::publishClock() {
  vrep_common::simRosGetInfo info;
    
  if(service.call(info)) {
    ros::Time simTime(info.response.simulationTime);
    rosgraph_msgs::Clock clockTime;
    clockTime.clock = simTime;
    if(clockTime.clock == clock.clock)
      clockTime.clock = ros::Time(0);
    else
      clock.clock = clockTime.clock;
    publisher.publish(clockTime);
  }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, NODE_NAME);

	ROSnode mNode;
	if(!mNode.Prepare())
		return -1;

	while (ros::ok()) {
		mNode.publishClock();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	return (0);
}
