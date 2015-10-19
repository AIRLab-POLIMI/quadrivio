/*
 * MsgToQuad.cpp
 *
 *  Created on: 18/apr/2011
 *      Author: simone
 */

#include "MsgToQuad.h"
#include <ostream>
#include <sstream>
#include <stdio.h>
#include <iomanip>

#define FORMAT_STR "<%011.4g,%011.4g,%011g,%011g,%011d>"
//#define MACRO_SET_SS std::width(11) <<

namespace QuadLib {

MsgToQuad::MsgToQuad() : 	brake(0), speed(0), steerAngle(0), steerSpeed(0), command(0){
}

std::string MsgToQuad::toString(){
	std::ostringstream ss;
	ss << "<";
	ss.width(11);
	ss.setf(ss.internal);
	ss.fill('0');
	ss << speed << "," ;
	ss.width(11);
	ss << brake << "," ;
	ss.width(11);
	ss << steerAngle << "," ;
	ss.width(11);
	ss << steerSpeed << "," ;
	ss.width(11);
	ss << command << ">" ;
	ss << "\n";
	return ss.str();
}


MsgToQuad::~MsgToQuad() {
}

}
