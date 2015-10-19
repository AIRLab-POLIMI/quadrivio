#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "roamros_msgs/SingleTrackAckermannOdometryStamped.h"
#include "Kinematic.h"
#include "boost/numeric/odeint.hpp"

#define NODE_NAME "fastPredictor"
#define KSPEED 0.0584
#define KSTEER -0.5622
#define L 1.25
#define PSI -0.0061

using namespace boost::numeric::odeint;

class ROSnode {
private:
	unsigned long seq;
	int size, absFrequency;
	Interpolator speedInt, steerInt;
	tf::TransformListener listener;
	tf::TransformBroadcaster broadcaster;
	ros::NodeHandle Handle;
	ros::Subscriber subscriber;
	std::string frame;
	void odoCallback(const roamros_msgs::SingleTrackAckermannOdometryStamped::ConstPtr& msg);
public:
  int frequency;
	bool Prepare();
	void estimatePosition();
};

bool ROSnode::Prepare() {
	//Retrieve parameters
	if (Handle.getParam("/reference/frequency", absFrequency))
		ROS_INFO("Node %s: retrieved parameter absFrequency.", ros::this_node::getName().c_str());
	else {
		ROS_FATAL("Node %s: unable to retrieve parameter absFrequency.", ros::this_node::getName().c_str());
		return false;
	}
	if (Handle.getParam("/fastPredictor/frequency", frequency))
		ROS_INFO("Node %s: retrieved parameter frequency.", ros::this_node::getName().c_str());
	else {
		ROS_FATAL("Node %s: unable to retrieve parameter frequency.", ros::this_node::getName().c_str());
		return false;
	}
	if (Handle.getParam("/fastPredictor/frame", frame))
		ROS_INFO("Node %s: retrieved parameter frame.", ros::this_node::getName().c_str());
	else {
		ROS_FATAL("Node %s: unable to retrieve parameter frame.", ros::this_node::getName().c_str());
		return false;
	}
  size = (frequency / absFrequency) + 3;
	seq = 0;

	//Init subscribers
	subscriber = Handle.subscribe("/odometry", 10, &ROSnode::odoCallback, this);

	speedInt = Interpolator(size);
	steerInt = Interpolator(size);

	/*do {
		ROS_INFO("Waiting for transformation");
	} while(!listener.waitForTransform("/world", "/"+frame, ros::Time::now(), ros::Duration(10)));*/

	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
	return true;
}

void ROSnode::odoCallback(const roamros_msgs::SingleTrackAckermannOdometryStamped::ConstPtr& msg) {
	speedInt.update(msg->odometry.speed, msg->header.stamp.toSec());
	steerInt.update(msg->odometry.steer, msg->header.stamp.toSec());
}

void ROSnode::estimatePosition() {
	if((listener.canTransform("/world", "/"+frame, ros::Time(0))) && (speedInt.canInterpolate() && steerInt.canInterpolate())) {
			tf::StampedTransform pose;
			listener.lookupTransform("/world", "/"+frame, ros::Time(0), pose);
			runge_kutta4<state_type> stepper;
			state_type x = {0.0, 0.0};
			ros::Time time = ros::Time::now() + ros::Duration(1/(frequency*2));
			integrate_const(stepper, Kinematic(speedInt, steerInt, KSPEED, KSTEER, L, PSI), x, pose.stamp_.toSec() , time.toSec() , 0.01);
			broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(x[1]), tf::Vector3(x[0],0,0)), time, frame, "base_link"));
	} else {
			broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(0), tf::Vector3(0,0,0)), ros::Time::now(), frame, "base_link"));
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, NODE_NAME);

	ROSnode mNode;
	if(!mNode.Prepare())
		return -1;

	ros::Rate loopRate(mNode.frequency);

	while (ros::ok()) {
		mNode.estimatePosition();
		ros::spinOnce();
		loopRate.sleep();
	}

	return (0);
}
