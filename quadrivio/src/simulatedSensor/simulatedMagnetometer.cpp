//This node simulates a magnetometer
//Topic published: /magnetic
//Message: sensor_msgs/MagneticField
//Topic subscribed: /vrep/simuMag
//Message: geometry_msgs/PoseStamped
//Parameters: x, y, z - magnetic field components
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/tf.h>

#define NAME_OF_THIS_NODE "simulatedMagnetometer"

using namespace tf;

class ROSnode {
private:
  ros::NodeHandle Handle;
  ros::Subscriber Subscriber;
  ros::Publisher Publisher;
  Matrix3x3 h, R;
  std::vector<double> S;
  long seq; 
  void simuMagCallback(const geometry_msgs::PoseStamped::ConstPtr& mag);  
public:
  bool Prepare();
};

bool ROSnode::Prepare() {
  std::vector<double> _h, _R;
  
  if (!Handle.getParam("/simulatedMagnetometer/magneticField", _h)) {
    ROS_FATAL("Parameter h undefined");
    return false;
  }
  h = Matrix3x3(_h[0], 0, 0, _h[1], 0, 0, _h[2], 0, 0);
  if (!Handle.getParam("/simulatedMagnetometer/R", _R)) {
    ROS_FATAL("Parameter R undefined");
    return false;
  }
  R = Matrix3x3(_R[0], _R[1], _R[2], _R[3], _R[4], _R[5], _R[6], _R[7], _R[8]);
  if (!Handle.getParam("/simulatedMagnetometer/S", S)) {
    ROS_FATAL("Parameter S undefined");
    return false;
  }
  //Retrieve parameters
  /*if (Handle.getParam("/simulatedMagnetometer/EarthMagneticField/x", magX)) {
    ROS_INFO("Node %s: retrieved parameter x.", ros::this_node::getName().c_str());
  }
  else {
    ROS_FATAL("Node %s: unable to retrieve parameter y.", ros::this_node::getName().c_str());
    return false;
  }

  if (Handle.getParam("/simulatedMagnetometer/EarthMagneticField/y", magY)) {
    ROS_INFO("Node %s: retrieved parameter y.", ros::this_node::getName().c_str());
  }
  else {
    ROS_FATAL("Node %s: unable to retrieve parameter x.", ros::this_node::getName().c_str());
    return false;
  }
  
  if (Handle.getParam("/simulatedMagnetometer/EarthMagneticField/z", magZ)) {
    ROS_INFO("Node %s: retrieved parameter z.", ros::this_node::getName().c_str());
  }
  else {
    ROS_FATAL("Node %s: unable to retrieve parameter x.", ros::this_node::getName().c_str());
    return false;
  }*/
  
  //Init publisher and subscriber
  Subscriber = Handle.subscribe("vrep/simuMag", 10, &ROSnode::simuMagCallback, this);
  Publisher = Handle.advertise<sensor_msgs::MagneticField>("magnetic", 20);
  
  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
  seq = 0;
  
  return true;
}

void ROSnode::simuMagCallback(const geometry_msgs::PoseStamped::ConstPtr& mag) {
  sensor_msgs::MagneticField msg;
  msg.header.seq = seq;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/imu";
  //Create a quaternion from the orientation
  Quaternion q = Quaternion(mag->pose.orientation.x, mag->pose.orientation.y, mag->pose.orientation.z, mag->pose.orientation.w);
  //Create a matrix from the quaternionS
  Matrix3x3 mtx = Matrix3x3(q);
  mtx = mtx.inverse();
  //Multipliy the matrix and create the vector
  mtx = R*mtx*h;
  msg.magnetic_field.x = mtx.getRow(0).getX() + S[0];
  msg.magnetic_field.y = mtx.getRow(1).getX() + S[1];
  msg.magnetic_field.z = mtx.getRow(2).getX() + S[2]; 
  boost::array<double, 9> covariance = {0};
  covariance[0] = 1;
  covariance[4] = 1;
  covariance[8] = 1;
  msg.magnetic_field_covariance = covariance;

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
