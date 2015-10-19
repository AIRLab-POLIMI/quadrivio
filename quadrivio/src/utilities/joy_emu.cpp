#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

#define KEYCODE_HALT 0x31
#define KEYCODE_SAFE 0x32
#define KEYCODE_AUTO 0x33
#define KEYCODE_ASSI 0x34
#define KEYCODE_MANU 0x35
#define KEYCODE_A 0x61
#define KEYCODE_B 0x62
#define KEYCODE_X 0x78
#define KEYCODE_Y 0x79
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42

class JoyEmu
{
public:
  JoyEmu();
  void keyLoop();
  void watchdog();
private:
	ros::NodeHandle nh_;
	ros::Publisher joy_pub_;
  ros::Time first_publish_;
  ros::Time last_publish_;
  boost::mutex publish_mutex_;
	int seq;
};

JoyEmu::JoyEmu() {
	joy_pub_ = nh_.advertise<sensor_msgs::Joy>("joy", 1);
	seq = 0;
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig) {
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "joy_emu");
	JoyEmu joy_emu;
	ros::NodeHandle n;
	signal(SIGINT,quit);
	boost::thread my_thread(boost::bind(&JoyEmu::keyLoop, &joy_emu));
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&JoyEmu::watchdog, &joy_emu));
	ros::spin();
	my_thread.interrupt() ;
	my_thread.join() ;
	return(0);
}

void JoyEmu::watchdog() {
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) && (ros::Time::now() > first_publish_ + ros::Duration(0.20))) {
    sensor_msgs::Joy msg;
		msg.header.stamp = ros::Time().now();
		msg.header.seq = seq;
		msg.buttons.resize(11);
		msg.axes.resize(8);
    joy_pub_.publish(msg);
		seq++;    
  }
}

void JoyEmu::keyLoop() {
	char c;
	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);
	puts("Reading from keyboard");
	puts("---------------------------");
	while (ros::ok()) {
		// get the next event from the keyboard
		if(read(kfd, &c, 1) < 0) {
			perror("read():");
			exit(-1);
		}
		
		ROS_INFO("value: 0x%02X\n", c);
		
		sensor_msgs::Joy msg;
		msg.header.stamp = ros::Time().now();
		msg.header.seq = seq;
		msg.buttons.resize(11);
		msg.axes.resize(8);
		
		switch(c) {
		case KEYCODE_HALT:
			ROS_INFO("HALT");
			msg.buttons[6] = 1;
			msg.buttons[7] = 1;
			break;
    case KEYCODE_SAFE:
      ROS_INFO("SAFE");
			msg.buttons[4] = 1;
			break;
    case KEYCODE_AUTO:
      ROS_INFO("AUTO");
      msg.axes[2] = -1;
      msg.axes[5] = -1;
      break;
    case KEYCODE_ASSI:
      ROS_INFO("ASSISTED");
      msg.buttons[5] = 1;
      break;
    case KEYCODE_MANU:
      ROS_INFO("MANUAL");
      msg.buttons[1] = 1;
      break;
    case KEYCODE_A:
      ROS_INFO("A");
      msg.buttons[0] = 1;
      break;
    case KEYCODE_B:
      ROS_INFO("B");
      msg.buttons[1] = 1;
      break;
    case KEYCODE_X:
      ROS_INFO("X");
      msg.buttons[2] = 1;
      break;
    case KEYCODE_Y:
      ROS_INFO("X");
      msg.buttons[3] = 1;
      break;
    case KEYCODE_U:
      ROS_INFO("SPEED");
      msg.axes[1] = 0.75;
      break;
    case KEYCODE_D:
      ROS_INFO("BRAKE");
      msg.axes[1] = -1.0;
      break;
    case KEYCODE_L:
      ROS_INFO("LEFT");
      msg.axes[3] = 1.0;
      break;
    case KEYCODE_R:
      ROS_INFO("RIGHT");
      msg.axes[3] = -1.0;
      break;
		};
    
    boost::mutex::scoped_lock lock(publish_mutex_);
    if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) {
      first_publish_ = ros::Time::now();
    }
    
    last_publish_ = ros::Time::now();
		
		joy_pub_.publish(msg);
		seq++;
	}
	return;
}
