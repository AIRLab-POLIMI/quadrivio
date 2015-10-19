//This node convert the raw input from the joy node to command
//It creates set points and controls the state machine
//Topic published: /setpoint
//Message: quadrivio/SetPoint
//Topic subscribed: /joy
//Message: sensor_msgs/Joy
//Parameters: max_speed - value of speed with a fully tilted analog
//            max_steer - value of steer with a fully tilted analog
#include <ros/ros.h>
#include <quadrivio_msgs/SetPoint.h>
#include <sensor_msgs/Joy.h>
#include <heartbeat/HeartbeatClient.h>

#define NAME_OF_THIS_NODE "joypad_translate"
#define JOY_SIZE 11

class ROSnode {
private:
  ros::NodeHandle Handle;
  ros::Subscriber Subscriber;
  ros::Publisher Publisher;
  double maxSpeed;
  double maxSteer;
  long seq;
  //Save last input read from the topic
  std::vector<int> lastJoyInput;
  //True when BACK and START are both pressed
  bool doubleButton;
  //state machine client
  HeartbeatClient *hb;
  void joypadCallback(const sensor_msgs::Joy::ConstPtr& joy);  
public:
  bool Prepare();
};

bool ROSnode::Prepare() {
  //Retrieve parameters
  if (Handle.getParam("/joypad_translate/max_speed", maxSpeed)) {
    ROS_INFO("Node %s: retrieved parameter max_speed.", ros::this_node::getName().c_str());
  }
  else {
    ROS_FATAL("Node %s: unable to retrieve parameter max_speed.", ros::this_node::getName().c_str());
    return false;
  }
  if (Handle.getParam("/joypad_translate/max_steer", maxSteer)) {
    ROS_INFO("Node %s: retrieved parameter max_steer.", ros::this_node::getName().c_str());
  }
  else {
    ROS_FATAL("Node %s: unable to retrieve parameter max_steer.", ros::this_node::getName().c_str());
    return false;
  }
  //Init publisher and subscriber
  Subscriber = Handle.subscribe("joy", 10, &ROSnode::joypadCallback, this);
  Publisher = Handle.advertise<quadrivio_msgs::SetPoint>("setpoint", 20);

  //Init state machine client
  hb = new HeartbeatClient(Handle);
  hb->start();

  seq = 0;
  doubleButton = false;
  lastJoyInput = std::vector<int>(JOY_SIZE, 0);
  
  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
  
  return true;
}

void ROSnode::joypadCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  heartbeat::State::_value_type state;
  quadrivio_msgs::SetPoint msg;
  //Get the current state and check its value
  state = hb->getState();
  switch(state) {
    //From every non-HALT state to HALT
    //BACK + START
  case heartbeat::State::AUTO:
  case heartbeat::State::SAFE:
  case heartbeat::State::MANUAL:
  case heartbeat::State::ASSISTED:
    if((joy->buttons[6] == 0 && joy->buttons[7] == 0) && doubleButton) {
      hb->setState(heartbeat::State::HALT);
      doubleButton = false;
      return;
    }
  } 
  switch(state) {
  case heartbeat::State::HALT:
    //HALT --> SAFE
    //BACK + START
    if((joy->buttons[6] == 0 && joy->buttons[7] == 0) && doubleButton) {
      hb->setState(heartbeat::State::SAFE);
      doubleButton = false;
      return;
    }
    break;
  case heartbeat::State::SAFE:
    //From SAFE
    //--> AUTO (LT + RT)
    if(joy->axes[2] == -1 && joy->axes[5] == -1) {
      hb->setState(heartbeat::State::AUTO);
      return;
    }
    //--> ASSISTED (RB)
    if(joy->buttons[5] == 0 && lastJoyInput[5] == 1) {
      hb->setState(heartbeat::State::ASSISTED);
      lastJoyInput = std::vector<int>(JOY_SIZE, 0);
      return;
    }
    //--> MANUAL (B)
    if(joy->buttons[1] == 0 && lastJoyInput[1] == 1) {
      hb->setState(heartbeat::State::MANUAL);
      lastJoyInput = std::vector<int>(JOY_SIZE, 0);
      return;
    }
    break;
  case heartbeat::State::AUTO:
    //AUTO --> SAFE
    //LB
    if(joy->buttons[4] == 0 && lastJoyInput[4] == 1) {
      lastJoyInput = std::vector<int>(JOY_SIZE, 0);
      hb->setState(heartbeat::State::SAFE);
      return;
    }
    break;
  case heartbeat::State::MANUAL:
  case heartbeat::State::ASSISTED:
    // MANUAL/ASSISTED --> SAFE
    //LB
    if(joy->buttons[4] == 0 && lastJoyInput[4] == 1) {
      lastJoyInput = std::vector<int>(JOY_SIZE, 0);
      hb->setState(heartbeat::State::SAFE);
      return;
    }
    //Veichle control

    /* header has been removed from SetPoint message
    msg.header.seq = seq;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/world";
    */
    
    //left analog up/down
    if(joy->axes[1] > 0) {
      msg.speed = maxSpeed * joy->axes[1];
      msg.brake = 0;
    }
    else {
      msg.speed = 0;
      msg.brake = -(100*joy->axes[1]);
    } 
    //right analog up/down
    msg.steer = maxSteer * joy->axes[3];
    msg.source = quadrivio_msgs::SetPoint::JOYPAD;
    Publisher.publish(msg);
    seq++;
    break;
  }
  //Save the input
  lastJoyInput = std::vector<int>(joy->buttons);
  if(joy->buttons[6] == 1 && joy->buttons[7] == 1) {
    doubleButton = true;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  ROSnode node;
  
  if(!node.Prepare())
    return 1;
  
  ros::spin(); 
  
  return (0);
}
