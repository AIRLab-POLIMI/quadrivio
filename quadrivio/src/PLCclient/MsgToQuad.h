/*
 * MsgToQuad.h
 *
 *  Created on: 18/apr/2011
 *      Author: simone
 */

#ifndef MSGTOQUAD_H_
#define MSGTOQUAD_H_

#include <string>

namespace QuadLib {

class MsgToQuad {
	float brake;
	float speed;
	float steerAngle;
	float steerSpeed;
	int command;      // added to manage the PLC state machine  

public:
	MsgToQuad();

	inline void setBrake(float brake){
		this->brake = brake;
	}
	inline void setSpeed(float speed){
		this->speed = speed;
	}
	inline void setSteerAngle(float steerAngle){
		this->steerAngle = steerAngle;
	}
	inline void setSteerSpeed(float steerSpeed){
		this->steerSpeed = steerSpeed;
	}
	
	inline void setCommand(int command){
	  this->command = command;
	}
	
	inline float getSpeed() { return this->speed; }
	inline float getBrake() { return this->brake; }
	inline float getSteer() { return this->steerAngle; }
	inline int getCommand() { return this->command; }

	std::string toString();

	virtual ~MsgToQuad();
};

}

#endif /* MSGTOQUAD_H_ */
