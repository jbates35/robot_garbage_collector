#pragma once

#include "CControl_Pi.h"

#include <vector>

#define _SERVOINT 	0.15
#define _SERVOINC 	15

enum {
	SCLOSE = 0,
	SOPEN
};

/**
*    @brief Servo
*    @details Implements a means of generating a servo object that can simply be commanded to open or close
*    @author Jimmy Bates
*    @date April 30 2021
*/
class servo
{
public:
	/**
	 * @brief Servo constructor
	 * **/
	servo();
	~servo();

	/**
	 * @brief Sets pin, open value, close value, and the initial state
	 * @param pin Pin servo is on
	 * @param sopen Value servo will be open to
	 * @param sclose Value servo will be closed to
	 * @param state SOPEN if initial state open, else CLOSE
	 * **/
	void init(int pin, int sopen, int sclose, bool state);

	/**
	 * @brief Open servo to open value
	 * @return None
	 * **/
	void open();

	/**
	 * @brief Close servo to close value
	 * @return None
	 * **/
	void close();

	/**
	 * @brief Returns whether servo is open (true) or closed (false)
	 * @return State of servo
	 * **/
	bool isopen() { return _servoState; }

	/**
	 * @brief Returns whether servo is stopped (true) or engaged/moving (false)
	 * @return State of servo
	 * **/
	bool isstopped() { return _servoReady; }

private:
	CControl_Pi _ctrl;
	int _openVal;
	int _closeVal;
	int _pin;
	bool _servoState;
	bool _servoReady;
	bool _servoState_prev;
	bool _thread_exit;
	int _servoVal;
	double _servoTime;

	void start();
	static void engage_thread(servo* ptr);
	void engage();
	
	//for ccontrol pi
	std::vector<int> _input;
	std::vector<int> _inputtype;
	std::vector<int> _button;
	std::vector<int> _output;
	std::vector<int> _outputtype;
};

