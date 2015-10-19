//This node show the current state of the state machine
//It reads the state and create a message for the led screen
//It creates a message every transition and periodically
//Topic published: /dcled
//Message: quadrivio/dcled
#include <ros/ros.h>
#include <quadrivio_msgs/dcled.h>
#include <heartbeat/HeartbeatClient.h>

#define NAME_OF_THIS_NODE "stateShower"

class ROSnode {
private:
  ros::NodeHandle Handle;
  ros::Publisher Publisher;
  ros::Timer timer;
  heartbeat::State::_value_type lastState;
  //state machine client
  HeartbeatClient *hb;  
public:
  bool Prepare();
  void checkState();
  void timeoutCallback(const ros::TimerEvent& event);
};

bool ROSnode::Prepare() {
  Publisher = Handle.advertise<quadrivio_msgs::dcled>("dcled", 20);

  //Init state machine client
  hb = new HeartbeatClient(Handle);
  hb->start();
  lastState = hb->getState();

  //Init timer for showing the state periodically
  timer = Handle.createTimer(ros::Duration(3), &ROSnode::timeoutCallback, this, false);
  
  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
  return true;
}

//send a message with the current state every T seconds
void ROSnode::timeoutCallback(const ros::TimerEvent& event) {
  std::stringstream ss;
  quadrivio_msgs::dcled msg;
  msg.priority = 0;
  switch(lastState) {
  case heartbeat::State::AUTO:
    ss << "AUTO";
    break;
  case heartbeat::State::SAFE:
    ss << "SAFE";
    break;
  case heartbeat::State::MANUAL:
    ss << "MANUAL";
    break;
  case heartbeat::State::ASSISTED:
    ss << "ASSISTED";
    break;
  case heartbeat::State::HALT:
    ss<< "HALT";
    break;
  }
  msg.message = ss.str();
  Publisher.publish(msg);
}

void ROSnode::checkState() {
  heartbeat::State::_value_type curState;
  std::stringstream ss;
  //get the current state, if different from the last, publish it
  curState = hb->getState();
  if(curState != lastState) {
    timer.stop();
    lastState = curState;
    quadrivio_msgs::dcled msg;
    msg.priority = 0;
    switch(lastState) {
    case heartbeat::State::AUTO:
      ss << "AUTO";
      break;
    case heartbeat::State::SAFE:
      ss << "SAFE";
      break;
    case heartbeat::State::MANUAL:
      ss << "MANUAL";
      break;
    case heartbeat::State::ASSISTED:
      ss << "ASSISTED";
      break;
    case heartbeat::State::HALT:
      ss<< "HALT";
      break;
    }
    msg.message = ss.str();
    Publisher.publish(msg);
    timer.start();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  ROSnode node;
  ros::Rate loopRate(15);
  
  if(!node.Prepare())
    return 1;
  
  //check the state periodically
  while(ros::ok()) {
    node.checkState();
    ros::spinOnce();
    loopRate.sleep();
  }

  return (0);
}
