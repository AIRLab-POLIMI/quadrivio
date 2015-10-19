/*
 * StringUtils.h
 *
 *  Created on: 19/apr/2011
 *      Author: simone
 */

#ifndef STRINGUTILS_H_
#define STRINGUTILS_H_

#include <string>
#include <vector>
#include <string>


namespace QuadLib {

class StringUtils {
public:
	static void tokenize(const std::string& str,
				  std::vector<std::string>& tokens,
				  const std::string& delimiters = " ");

};

}

#endif /* STRINGUTILS_H_ */
