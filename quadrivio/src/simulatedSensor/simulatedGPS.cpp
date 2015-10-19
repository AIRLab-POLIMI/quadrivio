//This node simulates a GPS
//Topic published: /enu
//Message: geometry_msgs/PoseWithCovarianceStamped
//Topic subscribed: /vrep/simuGPS
//Message: geometry_msgs/Point32
//Parameters: threshold - probability of a downtime (0 - 100)
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point32.h>



#define NAME_OF_THIS_NODE "simulatedGPS"

class ROSnode {
private:
  ros::NodeHandle Handle;
  ros::Subscriber Subscriber;
  ros::Publisher Publisher;
  ros::Timer timer, faultTime;
  int threshold;
  bool running;
  long seq;
  void simuGPSCallback(const geometry_msgs::Point32::ConstPtr& gps);
  void timeoutCallback(const ros::TimerEvent& event);
  void faultCallback(const ros::TimerEvent& event);
public:
  bool Prepare();
};

bool ROSnode::Prepare() {
  running = true;

  //Retrieve parameters
  if (Handle.getParam("/simulatedGPS/threshold", threshold)) {
    ROS_INFO("Node %s: retrieved parameter threshold.", ros::this_node::getName().c_str());
  }
  else {
    ROS_FATAL("Node %s: unable to retrieve parameter threshold.", ros::this_node::getName().c_str());
    return false;
  }

  //init subscriber and publisher
  Subscriber = Handle.subscribe("vrep/simuGPS", 10, &ROSnode::simuGPSCallback, this);
  Publisher = Handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("enu", 20);

  //Init timer for simulating a downtime
  timer = Handle.createTimer(ros::Duration(2.0), &ROSnode::timeoutCallback, this, false);
  faultTime = Handle.createTimer(ros::Duration(0.5), &ROSnode::faultCallback, this, false);
  
  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
  seq = 0;
  
  return true;
}

void ROSnode::timeoutCallback(const ros::TimerEvent& event) {
	double result = rand() % 100;
	if(result < threshold) {
		running = false;
		timer.stop();
		result = (rand() % 500) / 1000;
		faultTime.setPeriod(ros::Duration(0.75 + result));
		faultTime.start();
	}
}

void ROSnode::faultCallback(const ros::TimerEvent& event) {
	running = true;
	faultTime.stop();
	timer.start();
}

void ROSnode::simuGPSCallback(const geometry_msgs::Point32::ConstPtr& gps) {
  if (!running)
    return;
  //read the message and publish it as a PoseStamped message
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header.seq = seq;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/gps";
  msg.pose.pose.position.x = gps->x;
  msg.pose.pose.position.y = gps->y;
  msg.pose.pose.position.z = gps->z;
  boost::array<double, 36> covariance = {0};
  covariance[0] = 32.49;
  covariance[7] = 32.49;
  covariance[14] = 129.96;
  msg.pose.covariance = covariance;

  Publisher.publish(msg);
  seq++;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  ROSnode node;
  
  if(!node.Prepare())
    return 1;
  
  ros::spin(); 
  
  return (0);
}
