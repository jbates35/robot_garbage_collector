#pragma once


#include </usr/include/opencv2/opencv.hpp>

#include <string>
#include <sstream>
#include <vector>
#include <regex>
#include <thread>

#include <curses.h>

#include "pigpio.h"


#define SERVOPIN1           5          	// Pin for first servo

#define SERVOMIN1           92           	// Minimum of servo (open)
#define SERVOMAX1           145         	// Minimum of servo (closed)

#define SERVOMIN2           52         		// Minimum of servo (open)
#define SERVOMAX2           42         		// Minimum of servo (closed)

#define SERVOMIN3           57           	// Min of servo (TO THE LEFT)
#define SERVOMAX3           112         	// Max of servo (TO THE RIGHT)

#define LEDREDPIN          	19          // Pin for Boosterpack RED LED on Pi
#define LEDGRNPIN          	20          // Pin for Boosterpack GREEN LED on Pi
#define LEDBLUPIN          	21          // Pin for Boosterpack BLUE LED on Pi

#define DEBOUNCE           	0.1       // Debounce time for buttons
#define TIMEOUT            	0.15        // First timeout value after button is pressed
#define BUTTONINT          	0.1       // Interval (s) between continuous actuation (feedback)

#define PINCOUNT           40          // Pin count of MSP432

#define PWMA 27 // PWM A
#define PWMB 24 // PWM B
#define INA1 18 // MOTOR BOARD IN A1
#define INA2 17 // MOTOR BOARD IN A2
#define INB1 22 // MOTOR BOARD IN B1
#define INB2 23 // MOTOR BOARD IN B2

enum
{
	DIGITAL = 0,
	ANALOG,
	SERVO
};

enum
{
	EDGE = 0,
	LATCH
};

enum
{
	OFF = 0,
	ON
};

/**
*    @brief Serial communication w MSP432
*    @details This class is used to create an object that can init, set and get via Serial with the MSP432
*    @author Jimmy Bates
*    @version 0.1
*    @date Jan 25 2021
*/
class CControl_Pi
{
private:
	std::vector<int> rxData; ///<  Vector of latest data acknowledgement - 0 is type, 1 is channel, 2 is value
	int handle; ///< int of SPI Handle
	std::vector<bool> button; ///<  Flags to keep whether button is depressed
	std::vector<bool> released; ///<  Flags to keep whether button has been released
	std::vector<bool> contflag; ///< Flags to see if button is continuously pressed down
	std::vector<bool> updateFlag; ///<  Flags when the function has been polled successfully
	bool _ready; ///< Flag whether program is ready or not (if inputs have been successfully polled)
	std::vector<double> gb_timestart; ///<  Keeps track of when the get_button is activated
	std::vector<int> _input; ///< Vector of inputs to be monitored
	std::vector<int> _inputtype; ///< Vector of types
	bool _thread_exit; ///< Program will be polling the PI continuously while this is inactive

	/////////////////////////////////////////////////////////
	/// @brief Activates the update data thread
	/////////////////////////////////////////////////////////  
	void start();

	/////////////////////////////////////////////////////////
	/// @brief Tells the update data thread what to do
	/// @param ptr Pointer to be able to access information or instructions
	/////////////////////////////////////////////////////////  
	static void poll_inputs_thread(CControl_Pi* ptr);

	/////////////////////////////////////////////////////////
	/// @brief Polls through inputs
	/// @return true if output polling was completed
	/////////////////////////////////////////////////////////
	bool poll_inputs();

public:
	CControl_Pi();
	~CControl_Pi();

	/////////////////////////////////////////////////////////
	/// @brief Initializes the serial comm
	/// @param input vector of ints of inputs
	/// @param inputtype vector of ints of input types
	/// @param button vector of ints of buttons
	/// @param output vector of ints of outputs
	/// @param outputtype vector of ints of output types
	/// @return nothing to return
	/////////////////////////////////////////////////////////
	void init_com(std::vector<int> input, std::vector<int> inputtype, std::vector<int> button={}, std::vector<int> output={}, std::vector<int> outputtype={});

	/////////////////////////////////////////////////////////
	/// @brief Retrieves data from serial
	/// @param type If pin to get is analog, digital, or servo
	/// @param channel Pin number
	/// @param &result Value returned from microcontroller for pin
	/// @return true, false
	/////////////////////////////////////////////////////////
	bool get_data(int type, int channel, int& result);
	
	/////////////////////////////////////////////////////////
	/// @brief Makes sure program is ready
	/// @return flag if inputs have been polled
	/////////////////////////////////////////////////////////
	bool ready() { return _ready; }

	/////////////////////////////////////////////////////////
	/// @brief Writes data to serial
	/// @param type If pin to set is analog, digital, or servo
	/// @param channel Pin number
	/// @param value Value to set the pin in microcontroller
	/// @return true, false
	/////////////////////////////////////////////////////////
	bool set_data(int type, int channel, int val);

	/////////////////////////////////////////////////////////
	/// @brief Converts analogread -> ADC
	/// @param channel pin channel for the analogread
	/// @return int %value of ADC
	/////////////////////////////////////////////////////////
	int ADCCalc(int channel);

	/////////////////////////////////////////////////////////
	/// @brief Returns value of a button press with 100ms debounce
	/// @param comport port of port that will be used for serial communication
	/// @return nothing to return
	/////////////////////////////////////////////////////////
	bool get_button(int channel, bool conttype);

	/////////////////////////////////////////////////////////
	/// @brief Getter for latest value of an input or output
	/// @param channel Int of channel in data vector
	/// @return Value of channel 
	/////////////////////////////////////////////////////////
	int get_rxdata(int channel) { return rxData[channel]; }

}; // end Class CControl_Pi
