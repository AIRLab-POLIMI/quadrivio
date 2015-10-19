//This node simulates an IMU
//Topic published: /imu/data
//Message: sensor_msgs/Imu
//Topic subscribed:
// /vrep/imuData  - geometry_msgs/TwistStamped
//                  This message has a custom filling:
//                  msg.linear: linear acceleration
//                  msg.angular: angular velocity
// /vrep/realPose - geometry_msg/PoseStamped
#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

#define NAME_OF_THIS_NODE "simulatedImu"

class ROSnode {
private:
  ros::NodeHandle Handle;
  ros::Subscriber SubscriberSpeed;
  ros::Subscriber SubscriberPose;
  ros::Publisher Publisher;
  sensor_msgs::Imu msg;
  long seq; 
  void simuImuCallback(const geometry_msgs::TwistStamped::ConstPtr& mag);
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);
public:
  void publish();
  bool Prepare();
};

bool ROSnode::Prepare() {
  //Init publisher and subscriber
  SubscriberSpeed = Handle.subscribe("vrep/imuData", 10, &ROSnode::simuImuCallback, this);
  SubscriberPose = Handle.subscribe("vrep/realPose", 10, &ROSnode::poseCallback, this);
  Publisher = Handle.advertise<sensor_msgs::Imu>("imu/data", 20);
  
  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
  seq = 0;
  return true;
}

void ROSnode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose) {
  msg.orientation = pose->pose.orientation;
}

void ROSnode::simuImuCallback(const geometry_msgs::TwistStamped::ConstPtr& imu) {
  msg.header.seq = seq;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/imu";
  //identity matrix for the covariance
  boost::array<double, 9> covariance = {0};
  covariance[0] = 1;
  covariance[4] = 1;
  covariance[8] = 1;
  
  //fill the imu message
  msg.orientation_covariance = covariance;
  msg.angular_velocity = imu->twist.angular;
  msg.angular_velocity_covariance = covariance;
  msg.linear_acceleration = imu->twist.linear;
  msg.linear_acceleration_covariance = covariance;

  Publisher.publish(msg);
  seq++;  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  ROSnode node;
  
  if(!node.Prepare())
    return 1;
  
  ros::Rate loopRate(40);
  
  while (ros::ok()) {
    ros::spinOnce();
    loopRate.sleep();
  }   
  return (0);
}
