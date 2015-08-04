/**
 *  @file logger.hpp 
 *  @author Artur Wilkowski <ArturWilkowski@piap.pl>
 * 
 *  @section LICENSE
 *
 *  Copyright (C) 2015, Industrial Research Institute for Automation and Measurements
 *  Security and Defence Systems Division <http://www.piap.pl>
 */

#ifndef _LOGGER
#define _LOGGER

#include <string>
#include <vector>

class Logger {
protected:
	std::vector<std::string> fields ;
	std::string fileName ;
	size_t curr_index ;	
	bool loggingOn ;
public:
	Logger() ;
	Logger(const std::string &fileName) ;
	~Logger() ;
	void addField(const std::string &field) ;
	void nextRow() ;
	void initFile() ;
	void logHeader() ;
	template<typename T> void log(const std::string &field, const T value) ;
	void turnLoggingOn(bool loggingOn) ;
} ;

#endif
