#include "servo.h"

servo::servo()
{
}

servo::~servo()
{
}

void servo::init(int pin, int sopen, int sclose, bool state) 
{
    //Initial parameters
	_input = { 0 };
	_inputtype = { 0 };
	_button = { 0 };
	_output = { pin };
	_outputtype = { SERVO };

    //Creates object for being controlled by PiGPIO
	_ctrl.init_com(_input, _inputtype, _button, _output, _outputtype);

    //Initializes thread
	start();

	_servoTime=cv::getTickCount();
	_servoReady = true;
	_thread_exit=false;

	_pin = pin;
	_openVal = sopen;
	_closeVal = sclose;

    //Checks what state user wants to have servo initialized in whether open or close
	if(state) {
		_ctrl.set_data(SERVO, _pin, _openVal);
		_servoVal = _openVal;
		_servoState = SOPEN;
		_servoState_prev = SOPEN;
	} else {
		_ctrl.set_data(SERVO, _pin, _closeVal);
		_servoVal = _closeVal;
		_servoState = SCLOSE;
		_servoState_prev = SCLOSE;
	}
}

//Set flag to open
void servo::open()
{
	_servoState=SOPEN;
}

//Set flag to close
void servo::close()
{
	_servoState=SCLOSE;
}

//Just starts servo thread
void servo::start()
{
	std::thread t1(&servo::engage_thread, this);
   	t1.detach();
}

//While servo isn't destroyed, run loop that operates servo
void servo::engage_thread(servo* ptr)
{
   while (ptr->_thread_exit == false) {
      ptr->engage();
   }
}

//Function that operates servo
void servo::engage()
{
    //If there is a change in state, lock the servo so it can't take commands
	if(_servoState != _servoState_prev) {
		_servoState_prev = _servoState;
		_servoReady = false;
	}

    //Repeatedly check if state has been changed, if so, adjust servo to desired value
	if(!_servoReady) {
		if((cv::getTickCount() - _servoTime)/cv::getTickFrequency() >= 0.2) {
			_servoTime = cv::getTickCount();

			if(_servoState == SCLOSE) {
				_servoVal -= 9;
				if(_servoVal < _closeVal) _servoVal = _closeVal;
				if(_servoVal == _closeVal) _servoReady = true;
			} else {
				_servoVal += 9;
				if(_servoVal > _openVal) _servoVal = _openVal;
				if(_servoVal == _openVal) _servoReady = true;
			}

			_ctrl.set_data(SERVO, _pin, _servoVal);

		}
	}
}