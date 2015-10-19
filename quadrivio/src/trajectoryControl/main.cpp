/*
 * main.cpp
 *
 *  Created on: Dec 6, 2013
 *      Author: davide
 */

#include "ros/ros.h"

#include "TrajectoryControl.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectoryControl");

  TrajectoryControl node;

  if(!node.prepare())
    return 1;

  ros::Rate loopRate(20);
  
  while (ros::ok()) {
    node.newState();
    ros::spinOnce();
    loopRate.sleep();
  }

  return (0);
}

