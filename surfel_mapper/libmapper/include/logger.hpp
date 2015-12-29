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


/**
* @brief A class providing logging capabilities 
*
* The object of this class is used for logging certain outputs of the surfel mapper
* in order to provide experiments' evidence for further processing. The logger output
* is saved in CSV format.
*/
class Logger {
protected:
	std::vector<std::string> fields ; /**< Log field names */ 
	std::string fileName ; /**< Log file name */ 
	size_t curr_index ; /**< Current field index */ 
	bool loggingOn ; /**< Is logging turned on or off */
public:
	/**
	 * Logger default constructor. Initalizes logger with default 'log.csv' file name
	 */
	Logger() ;

	/**
	 * @brief Logger constructor.
	 *
	 * @param fileName log file name
	 */
	Logger(const std::string &fileName) ;

	/**
	 * @brief Logger destructor
	 */
	~Logger() ;

	/**
	 * @brief Registers field name for logging
	 * 
	 * @param field new field name
	 */
	void addField(const std::string &field) ;

	/**
	 * @brief Inserts a new line in the log file and resets field index
	 */
	void nextRow() ;

	/**
	 * @brief Initializes log file by writing a header
	 */
	void initFile() ;

	/**
	 * @brief Writes a header to the log file 
	 */
	void logHeader() ;

	/**
	 * @brief Writes an information associated with the given field name to the log file. 
	 *
	 * Consecutive columns are skipped until the specified field name is found.
	 *
	 * @param field field name
	 * @param value information to log
	 */
	template<typename T> void log(const std::string &field, const T value) ;

	/**
	 * @brief Turns logging on and off
	 *
	 * @param loggingOn true - turns logging on, false - turns logging off
	 */
	void turnLoggingOn(bool loggingOn) ;
} ;

#endif
