/*
 * MsgFromQuad.h
 *
 *  Created on: 19/apr/2011
 *      Author: simone
 */

#ifndef MSGFROMQUAD_H_
#define MSGFROMQUAD_H_

#include <string>
#include <vector>

namespace QuadLib {
class MsgFromQuad {
	bool valid;
	unsigned int time;
	float speed;
	float brake;
	float steer;
	int command;
  double fLT, lLT;

	std::vector<float> othersFields;

public:
	MsgFromQuad();

	bool fromString(const std::string & msgPar);

	inline bool isValid(){return valid;};

	inline unsigned int getTime(){return time;};
	inline float getSpeed(){return speed;};
	inline float getBrake(){return brake;};
	inline float getSteer(){return steer;};
  inline float getlLT(){return lLT;}; 
  inline float getfLT(){return fLT;};
  inline int getCommand(){return command;};

	inline unsigned int getOthersFieldSize() { return othersFields.size(); };
	inline float getOthersField(unsigned int i) { return othersFields[i]; };

	std::string toString();

	virtual ~MsgFromQuad();
};

}
#endif /* MSGFROMQUAD_H_ */
