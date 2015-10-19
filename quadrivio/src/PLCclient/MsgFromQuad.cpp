/*
 * MsgFromQuad.cpp
 *
 *  Created on: 19/apr/2011
 *      Author: simone
 */

#include "MsgFromQuad.h"
#include "StringUtils.h"

#include <sstream>
#include <stdlib.h>

//#define MSGFROMQUAD_DEBUG

#ifdef MSGFROMQUAD_DEBUG
#include <iostream>
#endif
namespace QuadLib {
MsgFromQuad::MsgFromQuad() : valid(false), time(0), speed(0), brake(0), steer(0), command(0){
}

bool MsgFromQuad::fromString(const std::string & msgPar){
	std::string msg = msgPar;
	othersFields.clear();
	valid = true;
	

	if(msg.size() < 2){
		valid = false;
#ifdef MSGFROMQUAD_DEBUG
		std::cerr << "empty or too short msg!!!" << std::endl;
#endif
	}
	if(valid){
		if(msg[0] != '<'){
			valid = false;
#ifdef MSGFROMQUAD_DEBUG
		std::cerr << "first char is not '<'" << std::endl;
#endif
		}
	}
	std::vector<std::string> tokens;
	if(valid){
		msg = msg.substr(1);
		
#ifdef MSGFROMQUAD_DEBUG
		std::cout << "remove message first char: " << msg << std::endl;
#endif
		QuadLib::StringUtils::tokenize(msg,tokens,",");
		if(tokens.size() < 5){
			valid = false;
#ifdef MSGFROMQUAD_DEBUG
		std::cout << "wrong number of tokens: less than 5 (" << tokens.size() << ")"<< std::endl;
#endif
		}
	}
	if(valid){
	  this->time = (unsigned int)atof(tokens[0].c_str());
	  this->steer = atof(tokens[2].c_str());
	  this->speed = atof(tokens[6].c_str());
	  this->brake = atof(tokens[11].c_str());
	  this->command = atoi(tokens[13].c_str());
	  //this->lLT = atof(tokens[5].c_str());
	  //this->fLT = atof(tokens[6].c_str());
	  
	  for(unsigned int i = 1; i < tokens.size(); i++){
	    othersFields.push_back(atof(tokens[i].c_str()));
	  }
	  valid = true;
	}
	return valid;
}

std::string MsgFromQuad::toString(){
	std::ostringstream ss;
	ss << "valid: "  << valid  << " ";
	ss << "time: "   << time   << " ";
	ss << "speed: "  << speed  << " ";
	ss << "brake: "  << brake  << " ";
	ss << "steer: "  << steer  << " ";
	ss << "command"  << command << " ";
	ss << "others : " ;
	for(unsigned int i = 0; i < othersFields.size(); i++){
		ss << othersFields[i] << " ";
	}
	return ss.str();
}


MsgFromQuad::~MsgFromQuad() {
}
}
