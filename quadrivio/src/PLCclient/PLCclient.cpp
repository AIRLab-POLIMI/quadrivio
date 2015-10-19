#include <mutex>

#include <ros/ros.h>

#include <ros/timer.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "tcp_ip_client.h"
#include "MsgToQuad.h"
#include "MsgFromQuad.h"

#include "quadrivio_msgs/SetPoint.h"
#include "quadrivio_msgs/QuadOdometry.h"
#include "quadrivio_msgs/PLCRawValues.h"

#include "heartbeat/HeartbeatClient.h"

#include "roamros_msgs/SingleTrackAckermannOdometryStamped.h"

struct sSetPoint {
  double speed;
  double steer;
  double brake;
};

// heartbeat client pointer
HeartbeatClient *_hb = NULL;

// mutex for concurrent access to the _lastSP variable
std::mutex _spMutex; //TODO: this throws an exception on destruction. Discover why
struct sSetPoint _lastSp = { 0, 0, 60.0 }; // variables which holds the last setpoint received from ROS
struct sSetPoint _errorSp = { 0, 0, 30.0 }; // TODO: this should go in the parameter server

// the tcp client class, 1024 is the buffer size and > is the packet terminator
QuadLib::TcpIpClient _client(1024, 1024, ">");

// ackermann constants, from roamfree
double _roamfreekSpeed;
double _roamfreekSteer;

//remember time of the last message
unsigned int lastTime = 0;
ros::Time lastROSTime;

void spCallback(const quadrivio_msgs::SetPoint& msg);
void timerCallback(const ros::TimerEvent& event);
void connectToPLC();

int main(int argc, char **argv) {

  // TCP IP settings
  std::string _quadPLCIpAddress;
  std::string _quadPLCPort;

  // frequency at which setpoints have to be sent at PLC
  double _frequency;

  // If I don't receive a PLC odometry for more that [s]
  // this is set into the heartbeat
  double _heratbeatTimeout;

  // return value for the last send request
  QuadLib::TcpIpReturnValue _lastTcpIpRecvRet;

  // -------------------------
  ros::init(argc, argv, "PLCclient");
  ros::NodeHandle n;

  // check the existance of needed parameters in the parameter server and initialize the local variables

  if (!n.getParam("/PLCclient/quadPLCIpAddress", _quadPLCIpAddress)) {
    ROS_FATAL("Parameter quadPLCIpAddress undefined");
    return 1;
  }

  if (!n.getParam("/PLCclient/quadPLCPort", _quadPLCPort)) {
    ROS_FATAL("Parameter quadPLCPort undefined");
    return 1;
  }

  if (!n.getParam("/PLCclient/frequency", _frequency)) {
    ROS_FATAL("Parameter frequency undefined");
    return 1;
  }

  if (!n.getParam("/PLCclient/heratbeatTimeout", _heratbeatTimeout)) {
    ROS_FATAL("Parameter heratbeatTimeout undefined");
    return 1;
  }

  if (!n.getParam("/PLCclient/kSpeed", _roamfreekSpeed)) {
    ROS_FATAL("Parameter kSpeed undefined");
    return 1;
  }

  if (!n.getParam("/PLCclient/kSteer", _roamfreekSteer)) {
    ROS_FATAL("Parameter kSteer undefined");
    return 1;
  }

  _hb = new HeartbeatClient(n, _heratbeatTimeout);
  _hb->start();

  // configure the TCP _client
  _client.setIp(_quadPLCIpAddress);
  _client.setTcpPort(_quadPLCPort);

  // subscribe to the setpoint topic
  ros::Subscriber sub = n.subscribe("setpoint", 16, spCallback);

  // advertise that I will publish odometry messages
  ros::Publisher _odoPub = n.advertise<quadrivio_msgs::QuadOdometry>("/PLC/odometry",
      16);
  ros::Publisher _odoAck = n.advertise<roamros_msgs::SingleTrackAckermannOdometryStamped>("/PLC/ackermann_odometry", 16);

  //advertise the publisher with the other values sent by the PLC
  ros::Publisher _otherPub = n.advertise<quadrivio_msgs::PLCRawValues>("/PLC/raw_values", 10);

  // timer which will trigger setpoints to be sent to the PLC
  ros::Timer sendTimer = n.createTimer(ros::Duration(1 / _frequency),
      timerCallback, false);

  // launch another thread to handle callbacks
  ros::AsyncSpinner spinner(1); // Use 4 threads
  spinner.start();

  // main loop. Keep trying to receive new messages
  while (ros::ok()) {
    if (_client.readMsg() == QuadLib::RETURN_OK) {

      // Process eventually received messages
      CharCircularBuffer *buffer = _client.getBuffer();

      unsigned int n = buffer->getLineCount();

      if (n > 0) {
        // TODO: decide a better I'm alive management.
        _hb->alive(); //send the "I'm alive" notification to heartbeat.

        char line[512];
        for (unsigned int i = 0; i < n; i++) {
          buffer->removeLine(line, 512);

          QuadLib::MsgFromQuad m;
          m.fromString(std::string(line));

          // determine message timestamp

          if (lastTime == 0) {
            //first message can't calculate dT
            lastROSTime = ros::Time::now();
          } else {
            lastROSTime = lastROSTime
                + ros::Duration((double)(m.getTime() - lastTime) / 1000);
          }

          lastTime = m.getTime();

          // publish a QuadOdometry message
          quadrivio_msgs::QuadOdometry msg;

          msg.header.stamp = lastROSTime;
	  msg.header.frame_id = "/base_link";

          msg.speed = m.getSpeed();
          msg.steer = m.getSteer();
          msg.brake = m.getBrake();

	  _odoPub.publish(msg);

          // publish also a SingleTrackAckermannOdometer message

          roamros_msgs::SingleTrackAckermannOdometryStamped ack_msg;

          ack_msg.header.stamp = lastROSTime;
	  ack_msg.header.frame_id = "/base_link";

          ack_msg.odometry.speed = m.getSpeed();
          ack_msg.odometry.steer = m.getSteer();

          _odoAck.publish(ack_msg);

          // publish other present values from PLC

          quadrivio_msgs::PLCRawValues vmg;

          vmg.time = m.getTime();

          vmg.steerSP = m.getOthersField(0);
	  vmg.steerPV = m.getOthersField(1);
	  vmg.steerCurrentSP = m.getOthersField(2);
	  vmg.steerCurrentPV = m.getOthersField(3);
	  vmg.speedSP = m.getOthersField(4);
	  vmg.speedPV = m.getOthersField(5);
	  vmg.throttleSP = m.getOthersField(6);
	  vmg.throttlePV = m.getOthersField(7);
	  vmg.throttleCS = m.getOthersField(8);
	  vmg.brakeSP = m.getOthersField(9);
	  vmg.brakePV = m.getOthersField(10);
	  vmg.brakeCS = m.getOthersField(11);
	  vmg.stateMachine = m.getOthersField(12);
          _otherPub.publish(vmg);

        }
      }
    }
  }

  return 0;
}

void timerCallback(const ros::TimerEvent& event) {
  QuadLib::MsgToQuad m;

  heartbeat::State::_value_type state = _hb->getState();

  switch (state) {
  case heartbeat::State::AUTO:
  case heartbeat::State::MANUAL:
  case heartbeat::State::ASSISTED:
    _spMutex.lock();
    m.setSpeed(_lastSp.speed / _roamfreekSpeed);
    m.setSteerAngle(_lastSp.steer / _roamfreekSteer);
    m.setBrake(_lastSp.brake);
    _spMutex.unlock();

    m.setCommand(40);
    break;

  case heartbeat::State::SAFE:
  case heartbeat::State::HALT:

    m.setSpeed(_errorSp.speed);
    m.setSteerAngle(_errorSp.steer);
    m.setBrake(_errorSp.brake);

    if (state == heartbeat::State::HALT) {
      m.setCommand(20);
    } else {
      m.setCommand(40);
    }

    break;
  }

  if (!_client.sendMsg(m.toString()) == QuadLib::RETURN_OK) {
    connectToPLC();
  }
}

void spCallback(const quadrivio_msgs::SetPoint& msg) {
  // get the current robot state
  heartbeat::State::_value_type state = _hb->getState();

  switch (state) {
  case heartbeat::State::AUTO:
    if (msg.source == quadrivio_msgs::SetPoint::AUTO) {
      _spMutex.lock();
      _lastSp.speed = msg.speed;
      _lastSp.steer = msg.steer;
      _lastSp.brake = msg.brake;
      _spMutex.unlock();
    }
    break;

  case heartbeat::State::ASSISTED:
  case heartbeat::State::MANUAL:
    if (msg.source == quadrivio_msgs::SetPoint::JOYPAD) {
      _spMutex.lock();
      _lastSp.speed = msg.speed;
      _lastSp.steer = msg.steer;
      _lastSp.brake = msg.brake;
      _spMutex.unlock();
    }

    break;
  }

  // we have to enqueue setPoints based on the state

}

void connectToPLC() {
  ROS_FATAL("Try to connection with PLC server ...");

  _client.closeConnection();
  if (_client.reConnect() == QuadLib::RETURN_OK) {
    ROS_FATAL("Success!");
  } else {
    ROS_FATAL("FAIL! :(");
  }
}
