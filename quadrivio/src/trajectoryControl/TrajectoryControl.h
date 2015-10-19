/*
 * TrajectoryControl.h
 *
 *  Created on: Dec 6, 2013
 *      Author: davide
 */

#ifndef TRAJECTORYCONTROL_H_
#define TRAJECTORYCONTROL_H_

#include <gsl/gsl_spline.h>
#include <tf/transform_listener.h>

#include "ros/ros.h"

#include "quadrivio_msgs/SetPoint.h"
#include "quadrivio_msgs/PathWithVelocity.h"
#include "quadrivio_msgs/ControllerInternal.h"
#include "geometry_msgs/PoseStamped.h"

#include "heartbeat/HeartbeatClient.h"

class TrajectoryControl {
public:

  TrajectoryControl();

  bool prepare();
  void newState();

protected:

  /** spline descriptor type */
  typedef struct {
    gsl_interp_accel *accumulator;
    gsl_spline *spline_coeff;
  } spline_param;

  /** this is the state which determines what to do when a new pose is received
   *
   * TODO: now
   *  DONOTHING -> FOLLOWING as soon as I receive a new set of waypoints
   *  FOLLOWING -> DONOTHING when I arrive at destination
   *
   */
  enum StateEnum {
    DONOTHING, FOLLOWING
  };
  StateEnum _state;

  /** publisher and subscriber handles */
  ros::Subscriber _pathSub;
  ros::Publisher _setpointPub;

  ros::Publisher _followingFrame;
  ros::Publisher _internalValues;

  tf::TransformListener listener;

  /** heartbeat client */
  HeartbeatClient *_hb;

  /** controller parameters */
  double _MAX_CURVATURE, _SOGLIA_ALPHA, _LAMBDA, _EPSILON, _GAMMA, _H, _BETA,
      _L;

  /** parameters for the velocity setpoint **/
  double _MAX_NEG_ACCEL; /** max negative acceleration in m/s^2 */
  double _BUTTER_LOSS; /** for the butter algorithm, amount of acceleration to redistribute */

  double _MIN_VEL; /** maximum velocity [m/s] */
  double _MAX_VEL; /** minimum velocity [m/s] */
  double _MIN_CURV; /** curvature @99% max_vel */
  double _MAX_CURV; /** curvature @101% min_vel */

  double _localizationNominalFreq;

  /** path spline parameter descriptos */
  spline_param tj_x, tj_y, sp;
  double path_length, s_act;

  ros::NodeHandle _node;

  void newPlan(const quadrivio_msgs::PathWithVelocity &msg);

  double angle_wrap(double angle);

  void DamelioFontanile_controller(double x, double y, double phi,
      double* s_path_act, spline_param x_path, spline_param y_path,
      spline_param ds_path, double dt, double lungh, double* v_ref,
      double* psi_ref);

  void pathSpline_destroy(spline_param* x_path, spline_param* y_path,
      spline_param* ds_path);

};

#endif /* TRAJECTORYCONTROL_H_ */
