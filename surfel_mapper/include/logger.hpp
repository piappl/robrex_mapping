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
