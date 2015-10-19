//This node generate a trajectory and publish it on a topic
//Command are isseud via joypad

//Topics published:
// /path           - Message: quadrivio_msgs/PathWithVelocity
//The complete recorded path
// /visualize_path - Message: nav_msgs/Path
//The partial path published periodically

//Topics subscribed:
// /joy - sensor_msgs/Joy

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <quadrivio_msgs/PathWithVelocity.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#define NODE_NAME "trajectoryGenerator"
#define PI 3.14159265

//Internal state of the node
enum State {
	WAIT, TRAJECTORY
};

class ROSnode {
private:
	unsigned long seqPath;
	unsigned long seqVPath;

	State curState;

	double deadZone;

	tf::TransformListener listener;

	ros::NodeHandle Handle;
	ros::Subscriber subJoypad;
	ros::Publisher pubPath;
	ros::Publisher pubVPath;
	ros::Publisher pubState;
	ros::Timer timer;

	nav_msgs::Path currentPath;
	std::vector<double> velocity;

	void readCommands();
	void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

	void timeoutCallback(const ros::TimerEvent& event);

public:
	bool Prepare();
};

bool ROSnode::Prepare() {
	seqPath = 0;
	seqVPath = 0;
	curState = WAIT;

	//Init subscribers
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

void ROSnode::readCommands() {
	using namespace std;
	using namespace boost;
	//get a pose, discard it if the state isn't correct
	if (curState == TRAJECTORY && currentPath.poses.size() == 0 ) {
		double speed;
		if (!Handle.getParam("/trajectoryGenerator/speed", speed))
			speed = 3.0;
		tf::StampedTransform start;
		listener.lookupTransform("/world", "/base_link", ros::Time(0), start);

		start.setData(start*tf::Transform(tf::createQuaternionFromYaw(0), tf::Vector3(1, 0, 0)));

		geometry_msgs::PoseStamped element;

		element.header.frame_id = "/world";
		element.pose.position.x = start.getOrigin().x();
		element.pose.position.y = start.getOrigin().y();
		element.pose.position.z = start.getOrigin().z();
		tf::quaternionTFToMsg(start.getRotation(), element.pose.orientation);

		currentPath.poses.push_back(element);
		velocity.push_back(speed);

		tf::Pose lastPose;
		tf::poseMsgToTF(element.pose, lastPose);

		ifstream cmds;
		string commands;
		if(!Handle.getParam("/trajectoryGenerator/commands", commands)) {
			const char* home = getenv("HOME");
			commands = home;
			commands += "/commands";
		}
		ROS_INFO("%s", commands.c_str());
		cmds.open(commands.c_str());
		if(cmds.is_open()) {
			ROS_INFO("file aperto");
			string str;
			vector <string> fields;
			getline(cmds, str);
			while (!cmds.eof()) {
				if(str[0] != '#') {
					ROS_INFO("%s", str.c_str());
					split(fields, str, is_any_of(","));
					if(str[0] == 'f') {
						switch (atoi(fields[1].c_str())) {
						case 0:
							double w = 1 / atof(fields[4].c_str());
							double A = atof(fields[3].c_str());
							for(double i = 0.01; i <= atof(fields[2].c_str());i = i + 0.01) {
								double y = A*cos(2*PI*w*i);
								tf::Transform command(tf::createQuaternionFromYaw(0), tf::Vector3(i, y - A, 0));
								//lastPose = lastPose*command;
								tf::poseTFToMsg(lastPose*command, element.pose);
								currentPath.poses.push_back(element);
								velocity.push_back(speed);
							}
							break;
						}


					} else {
						tf::Transform command(tf::createQuaternionFromYaw(atof(fields[1].c_str())), tf::Vector3(atof(fields[0].c_str()), 0, 0));
						int k = atoi(fields[2].c_str());
						for(int i = 0;i < k;i++) {
							lastPose = lastPose*command;
							tf::poseTFToMsg(lastPose, element.pose);
							currentPath.poses.push_back(element);
							velocity.push_back(speed);
						}
					}
				}
				getline(cmds, str);
			}
			cmds.close();
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
		if (msg->buttons[0] == 1) { // go to the trajectory state
			curState = TRAJECTORY;
			ss << "TRAJECTORY";
			str.data = ss.str();
			pubState.publish(str);
			ROS_INFO("Current State: TRAJECTORY");
			readCommands();
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
	if (curState == TRAJECTORY) {
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

	ros::spin();

	return (0);
}
