/*
 * trajectoryControl.cpp
 *
 *  Created on: Dec 6, 2013
 *      Author: davide
 */

#include "TrajectoryControl.h"

#include <cmath>

#include <fstream>

// some functions to unwrap angles, as with the matlab function unwrap
inline double constrainAngle(double x) {
	x = std::fmod(x + M_PI, 2 * M_PI);
	if (x < 0)
		x += 2 * M_PI;
	return x - M_PI;
}
// convert to [-360,360]
inline double angleConv(double angle) {
	return fmod(constrainAngle(angle), 2 * M_PI);
}

inline double angleDiff(double a, double b) {
	double dif = fmod(b - a + M_PI, 2 * M_PI);
	if (dif < 0)
		dif += 2 * M_PI;
	return dif - M_PI;
}

inline double unwrap(double previousAngle, double newAngle) {
	return previousAngle - angleDiff(newAngle, angleConv(previousAngle));
}

/* esegue il check per valutare se l'angolo sta tra + e - pi greco*/
double TrajectoryControl::angle_wrap(double angle) {
	if ((angle <= M_PI) && (angle >= -M_PI))
		return angle;
	else if (angle > 0)
		return angle - 2 * M_PI;
	else
		return angle + 2 * M_PI;
}

void TrajectoryControl::DamelioFontanile_controller(double x, double y,
		double phi, double* s_path_act, spline_param x_path, spline_param y_path,
		spline_param ds_path, double dt, double lungh, double* v_ref,
		double* psi_ref) {
	phi = fmod(phi, 2 * M_PI);

	// calcolo configurazione riferimento nella ascissa corrente
	double xr = gsl_spline_eval(x_path.spline_coeff, *s_path_act,
			x_path.accumulator);
	double yr = gsl_spline_eval(y_path.spline_coeff, *s_path_act,
			y_path.accumulator);
	double phir = atan2(
			gsl_spline_eval_deriv(y_path.spline_coeff, *s_path_act,
					y_path.accumulator),
			gsl_spline_eval_deriv(x_path.spline_coeff, *s_path_act,
					x_path.accumulator));

	/* calcolo e, alpha,theta*/
	double e = sqrt(pow(xr - x, 2) + pow(yr - y, 2));
	double theta = atan2(yr - y, xr - x) - phir;
	double alpha = theta + phir - phi;
	theta = angle_wrap(theta);
	alpha = angle_wrap(alpha);

	/* calcolo delle variabili di controllo*/
	// calcola velocita'
	*v_ref = fmin(_GAMMA * e,
			gsl_spline_eval(ds_path.spline_coeff, *s_path_act, ds_path.accumulator)); // e' gia' compresa la saturazione

	double b;
	if (abs(alpha) < _SOGLIA_ALPHA)
		b = 1;
	else
		b = sin(alpha) / alpha;
	double c = fmax(
			fmin(sin(alpha) / e + theta / e * b + _BETA * alpha / e, _MAX_CURVATURE),
			-_MAX_CURVATURE); // curvatura gia' saturata

	// calcola angolo di sterzo
	*psi_ref = atan(c * _L); // angolo di sterzo della ruota anteriore nel modello bicicletta

	/* calcolo della funzione V*/
	double V = _LAMBDA * (pow(e, 2)) + pow(alpha, 2) + _H * (pow(theta, 2));

	double S_punto;
	if (V > _EPSILON)
		S_punto = 0;
	else
		S_punto = (gsl_spline_eval(ds_path.spline_coeff, *s_path_act,
				ds_path.accumulator)) * (1 - V / _EPSILON);

	// Aggiorno la posizione sul path
	*s_path_act += (S_punto * dt) / lungh;

	/* debug, publish the following frame xr, yr, phir */

	geometry_msgs::PoseStamped ff;

	// TODO: put the timestamp of the incoming pose
	ff.header.stamp = ros::Time(0.0);
	ff.header.frame_id = "/world";

	// TODO: put the same z as the robot pose
	ff.pose.position.x = xr;
	ff.pose.position.y = yr;
	ff.pose.position.z = 0.0;

	ff.pose.orientation.w = cos(phir / 2.0);
	ff.pose.orientation.x = 0.0;
	ff.pose.orientation.y = 0.0;
	ff.pose.orientation.z = sin(phir / 2.0);

	_followingFrame.publish(ff);

	quadrivio_msgs::ControllerInternal msg;

	msg.header.stamp = ros::Time::now();
	msg.V = V;
	msg.S_punto = S_punto;
	msg.e = e;
	msg.theta = theta;
	msg.alpha = alpha;

	_internalValues.publish(msg);

	/* end of debug */
}

bool TrajectoryControl::prepare() {

	// load the controller parameters from the parameter server
	//double __MAX_CURVATURE, _SOGLIA_ALPHA, _LAMBDA, _EPSILON, _GAMMA, _H, _BETA,
	//    _L;

	if (!_node.getParam("/trajectoryControl/maxCurvature", _MAX_CURVATURE)) {
		ROS_FATAL("Parameter maxCurvature undefined");
		return false;
	}
	if (!_node.getParam("/trajectoryControl/sogliaAlpha", _SOGLIA_ALPHA)) {
		ROS_FATAL("Parameter sogliaAlpha undefined");
		return false;
	}
	if (!_node.getParam("/trajectoryControl/lambda", _LAMBDA)) {
		ROS_FATAL("Parameter lambda undefined");
		return false;
	}
	if (!_node.getParam("/trajectoryControl/epsilon", _EPSILON)) {
		ROS_FATAL("Parameter epsilon undefined");
		return false;
	}
	if (!_node.getParam("/trajectoryControl/gamma", _GAMMA)) {
		ROS_FATAL("Parameter gamma undefined");
		return false;
	}
	if (!_node.getParam("/trajectoryControl/H", _H)) {
		ROS_FATAL("Parameter H undefined");
		return false;
	}
	if (!_node.getParam("/trajectoryControl/beta", _BETA)) {
		ROS_FATAL("Parameter beta undefined");
		return false;
	}
	if (!_node.getParam("/trajectoryControl/max_negative_acceleration",
			_MAX_NEG_ACCEL)) {
		ROS_FATAL("Parameter max_negative_acceleration undefined");
		return false;
	}
	if (!_node.getParam("/trajectoryControl/butter_loss", _BUTTER_LOSS)) {
		ROS_FATAL("Parameter butter_loss undefined");
		return false;
	}

	if (!_node.getParam("/trajectoryControl/max_vel", _MAX_VEL)) {
		ROS_FATAL("Parameter max_vel undefined");
		return false;
	}
	if (!_node.getParam("/trajectoryControl/min_vel", _MIN_VEL)) {
		ROS_FATAL("Parameter min_vel undefined");
		return false;
	}
	if (!_node.getParam("/trajectoryControl/max_curv", _MAX_CURV)) {
		ROS_FATAL("Parameter max_curv undefined");
		return false;
	}
	if (!_node.getParam("/trajectoryControl/min_curv", _MIN_CURV)) {
		ROS_FATAL("Parameter min_curv undefined");
		return false;
	}

	std::vector<double> __L;
	if (!_node.getParam("/ackermannParameters/L", __L)) {
		ROS_FATAL("Parameter L undefined");
		return false;
	}
	_L = __L[0];

	_localizationNominalFreq = 20;

	// subscribe to topics
	_pathSub = _node.subscribe("/path", 1, &TrajectoryControl::newPlan, this);

	// set up the setpoint publisher
	_setpointPub = _node.advertise<quadrivio_msgs::SetPoint>("setpoint", 16);

	// set up the debug topic which contains the pose of the ref frame we are following
	_followingFrame = _node.advertise<geometry_msgs::PoseStamped>(
			"followingFrame", 16);

	_internalValues = _node.advertise<quadrivio_msgs::ControllerInternal>(
			"controllerInternal", 10);

	_hb = new HeartbeatClient(_node);
	_hb->start();

	return true;
}

void TrajectoryControl::newPlan(const quadrivio_msgs::PathWithVelocity &msg) {

	ROS_INFO("got new plan");

	// ----------  extract waypoint coordinates from input message

	// debug
	std::fstream g("x.txt", std::ios_base::out);
	g.precision(10);

	int N = msg.path.poses.size();
	double xx[N], yy[N], s_punto[N];

	for (int ii = 0; ii < N; ii++) {
		xx[ii] = msg.path.poses[ii].pose.position.x;
		yy[ii] = msg.path.poses[ii].pose.position.y;

		s_punto[ii] = msg.velocities[ii];

		g << xx[ii] << "," << yy[ii] << std::endl;
	}

	g.close();

	// ----------  destroy old splines
	if (tj_x.accumulator != NULL) {
		pathSpline_destroy(&tj_x, &tj_y, &sp);
	}

	// ----------  compute path length and build the s vector

	double s[N], d[N - 1];

	d[0] = sqrt(pow((xx[1] - xx[0]), 2) + pow((yy[1] - yy[0]), 2));
	for (int ii = 1; ii < N - 1; ii++) {
		d[ii] = d[ii - 1]
				+ sqrt(pow((xx[ii + 1] - xx[ii]), 2) + pow((yy[ii + 1] - yy[ii]), 2));
	}

	s[0] = 0;
	for (int ii = 1; ii < N; ii++) {
		s[ii] = d[ii - 1] / d[N - 2];
	}

	path_length = d[N - 2];

	//  ---------- compute max speed as a function of the local curvature

	// debug
	std::fstream f("v.txt", std::ios_base::out);

	// Velocity/curvature parameters
	/*
	 double _MAX_VEL = 4.0;  // maximum velocity [m/s]
	 double _MIN_VEL = 2.0;       // minimum velocity [m/s]
	 double _MIN_CURV = 0.1;     // curvature @99% _MAX_VEL
	 double _MAX_CURV = 0.5;     // curvature @101% _MIN_VEL
	 */

	double a1 = _MAX_VEL - _MIN_VEL;
	double a2;
	double a3 = (_MIN_CURV + _MAX_CURV) / 2;
	double a4 = _MIN_VEL;
	if (a4 != 0) {
		a2 = 1.0 / (_MAX_CURV - a3) * log(100.0 * a1 / a4 - 1.0);
	} else {
		a2 = 1.0 / (_MIN_CURV - a3) * log((a1 + a4) / (99.0 * a1 - a4));
	}

	// vector cointaining for each waypoint a max speed computed as a function
	// of the local trajectory curvature
	double max_speed[N];
	double cur_theta, last_theta = 0;

	for (int i = 1; i < N; i++) {
		last_theta = cur_theta;

		// compute segment angle wrt wrold
		double dx = xx[i] - xx[i - 1];
		double dy = yy[i] - yy[i - 1];

		cur_theta = unwrap(last_theta, std::atan2(dy, dx));

		// compute dtheta. The first time the value is wrong since last_theta
		// does not contain a correct value. We overwrite this outcome later

		double dtheta = std::fabs(cur_theta - last_theta);
		double kappa = std::sin(dtheta) / (d[i - 1] - (i > 1 ? d[i - 2] : 0.0)); // curvature

		max_speed[i - 1] = a1 / (1 + std::exp(a2 * (std::fabs(kappa) - a3))) + a4;

		f << cur_theta << ", " << dtheta << ", " << kappa << ", "
				<< max_speed[i - 1] << ", " << (d[i - 1] - (i > 1 ? d[i - 2] : 0.0))
				<< std::endl;
	}

	f.close();

	//overwrite the first and extend to the last
	max_speed[0] = max_speed[1];
	max_speed[N - 1] = max_speed[N - 2];

	//  ---------- apply the butter knife algorithm

	// I differentiate max_speed, iterating backwards, to obtain max_acceleration and
	// perform the butter thing.

	// debug
	std::fstream h("a.txt", std::ios_base::out);

	double acceleration[N - 1];

	double butter = 0.0;
	for (int i = N - 1; i > 0; i--) {
		double cur_max_acceleration = (max_speed[i] - max_speed[i - 1])
				/ (d[i - 1] - (i > 1 ? d[i - 2] : 0.0));

		if (cur_max_acceleration < _MAX_NEG_ACCEL / max_speed[i]) {
			butter = butter
					+ (_MAX_NEG_ACCEL / max_speed[i] - cur_max_acceleration)
							* (d[i - 1] - (i > 1 ? d[i - 2] : 0.0));

			acceleration[i] = _MAX_NEG_ACCEL / max_speed[i];
		} else {
			double drop = std::min(
					cur_max_acceleration - _MAX_NEG_ACCEL / max_speed[i],
					butter * _BUTTER_LOSS / (d[i - 1] - (i > 1 ? d[i - 2] : 0.0)));

			acceleration[i] = cur_max_acceleration - drop;
			butter -= drop * (d[i - 1] - (i > 1 ? d[i - 2] : 0.0));
		}

		h << cur_max_acceleration << ", " << acceleration[i] << ", " << butter
				<< std::endl;
	}

	h.close();

	// I might still have butter to drop, in that case we start slower
	if (butter > 0) {
		ROS_INFO("Need to reduce initial speed");
	}

	//  ---------- integrate the resulting acceleration to compute speed

	double ds[N];

	ds[0] = max_speed[0] - butter;

	for (int i = 1; i < N; i++) {
		ds[i] = ds[i - 1] + acceleration[i] * (d[i - 1] - (i > 1 ? d[i - 2] : 0.0));
	}

	// ---------- interpolate resulting speed with a spline

	// Interpolazione spline

	//debug
	std::fstream ss("s.txt", std::ios_base::out);

	for (int i = 0; i < N; i++) {
		ss << s[i] << ", " << xx[i] << ", " << yy[i] << ", " << ds[i] << ", "
				<< std::endl;
	}

	tj_x.accumulator = gsl_interp_accel_alloc();
	tj_x.spline_coeff = gsl_spline_alloc(gsl_interp_cspline, N);
	gsl_spline_init(tj_x.spline_coeff, s, xx, N);

	tj_y.accumulator = gsl_interp_accel_alloc();
	tj_y.spline_coeff = gsl_spline_alloc(gsl_interp_cspline, N);
	gsl_spline_init(tj_y.spline_coeff, s, yy, N);

	sp.accumulator = gsl_interp_accel_alloc();
	sp.spline_coeff = gsl_spline_alloc(gsl_interp_cspline, N);
	gsl_spline_init(sp.spline_coeff, s, ds, N);

	s_act = 0.0; // we are at the beginning
	_state = FOLLOWING;

}

TrajectoryControl::TrajectoryControl() :
		_state(DONOTHING), _hb(NULL) {
	tj_x.accumulator = NULL;
	tj_x.spline_coeff = NULL;

	tj_y.accumulator = NULL;
	tj_y.spline_coeff = NULL;

	sp.accumulator = NULL;
	sp.spline_coeff = NULL;
}

void TrajectoryControl::newState() {
	switch (_state) {
	case DONOTHING:
		ROS_INFO("do nothing...");
		// try to guess..
		break;
	case FOLLOWING:
		if (_hb->getState() != heartbeat::State::AUTO) {
			_state = DONOTHING;
			break;
		}
		tf::StampedTransform transform;
		listener.lookupTransform("/world", "/base_link", ros::Time(0), transform);

		// we have to determine the new setpoint
		double dt = 1.0 / _localizationNominalFreq;

		double x = transform.getOrigin().x();
		double y = transform.getOrigin().y();

		//this is for determining phi from the input quaternion
		double xR, yR; //x axis of the robot frame w.r.t. world frame
		double phi; // bearing
		xR = pow(transform.getRotation().w(), 2)
				+ pow(transform.getRotation().x(), 2)
				- pow(transform.getRotation().y(), 2)
				- pow(transform.getRotation().z(), 2);
		yR = 2
				* (transform.getRotation().x() * transform.getRotation().y()
						+ transform.getRotation().w() * transform.getRotation().z());
		phi = atan2(yR, xR);

		quadrivio_msgs::SetPoint outSp;
		outSp.brake = 0;

		DamelioFontanile_controller(x, y, phi, &s_act, tj_x, tj_y, sp, dt,
				path_length, &outSp.speed, &outSp.steer);

		/* --------- saturazione del setpoint di velocità - OBSOLETA
		 double sigmoid_y2 = 0.01, sigmoid_y1 = 0.9, sigmoid_x1 = 0.15, sigmoid_x2 =
		 0.6;
		 double maxsteerspeed = 2.2;

		 double c = std::log(
		 sigmoid_y1 * (1.0 - sigmoid_y2) / (sigmoid_y2 * (1.0 - sigmoid_y1)))
		 * 1.0 / (sigmoid_x2 - sigmoid_x1);

		 double b = (1.0 - sigmoid_y1) / sigmoid_y1 * std::exp(-c * sigmoid_x1);

		 if (outSp.speed > maxsteerspeed) {
		 double k_dump_speed = 1.0
		 / (1.0 + b * std::exp(c * std::fabs(outSp.steer)));
		 outSp.speed = k_dump_speed * (outSp.speed - maxsteerspeed)
		 + maxsteerspeed;
		 }
		 */

		if (s_act < 1.0) {
			ROS_INFO("pose: %g %g %g - sp: %g %g", x, y, phi, outSp.speed,
					outSp.steer);

		} else { //if we are at the end of the path
			ROS_INFO("we have reached the end of the plan");

			outSp.speed = 0;
			outSp.steer = 0;
			outSp.brake = 20;

			_state = DONOTHING;

			//TODO: trigger robot state transition
		}

		_setpointPub.publish(outSp);

		break;
	}
}

void TrajectoryControl::pathSpline_destroy(spline_param* x_path,
		spline_param* y_path, spline_param* ds_path) {
	// Distruggo gli oggetti spline
	gsl_spline_free(x_path->spline_coeff);
	gsl_interp_accel_free(x_path->accumulator);

	gsl_spline_free(y_path->spline_coeff);
	gsl_interp_accel_free(y_path->accumulator);

	gsl_spline_free(ds_path->spline_coeff);
	gsl_interp_accel_free(ds_path->accumulator);
}
