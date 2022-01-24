#include "robotDriver.h"

std::mutex drivemut;

robotDriver::robotDriver()
{
    //Initialize thread and other values
   start();
   _dir = STOP;
   _dir_prev = _dir;
   _dir_actual = STOP;
   PWM_scale = 0;

   _brakePrompt = false;
   _braked = false;
   PWM_halfright = 1;
   PWM_halfleft = 1;

   brakeTime = cv::getTickCount();
   accelTime = cv::getTickCount();
   msdelay = cv::getTickCount();

   //Initializing control for use by PiGPIO
   _ready.assign(15, false);
   _input = { 0 };
   _inputtype = { 0 };
   _output = { PWMA, PWMB, INA1, INA2, INB1, INB2 };
   _outputtype = { ANALOG, ANALOG, DIGITAL, DIGITAL, DIGITAL, DIGITAL };
   _button = { 0 };

   _ctrl.init_com(_input, _inputtype, _button, _output, _outputtype);

  //Makes sure pins are set before running script so we don't have a robot trailing off into the distance
   while (!_ready[0] || !_ready[1]) {
      if (_ctrl.set_data(ANALOG, PWMA, 100)) _ready[0] = true; // PWM_scale * PWM_MAX_A/100)) _ready[0] = true;
      if (_ctrl.set_data(ANALOG, PWMB, 100)) _ready[1] = true; // PWM_scale * PWM_MAX_B/100)) _ready[1] = true;
   }
}

robotDriver::~robotDriver()
{
}

int robotDriver::setDir(int dir)
{
   int returnVal = FAIL; // 0 = fail, 1 = success, 2 = busy

   //Possibly add script here to accomodate motors being busy changing direction
   //Alternative solution is to have 0.1s between any speed change
   drivemut.lock();
   _dir = dir;
   drivemut.unlock();

   if (_brakePrompt) {
      returnVal = BUSY;
   } else {
      if (_dir != _dir_prev) {
         _brakePrompt = true;
         _dir_prev = _dir;
         returnVal = SUCCESS;
      }
   }
   return returnVal;
}

int robotDriver::getDir()
{
   //drivemut.lock();
   return _dir_actual;
   //drivemut.unlock();
}

bool robotDriver::forward()
{
   bool returnValue = false;

   //Rotate wheels
   rotwheel(FORWARD, LEFT);
   rotwheel(FORWARD, RIGHT);

   return returnValue;
}

bool robotDriver::left()
{
   bool returnValue = false;

   //Rotate wheels
   rotwheel(FORWARD, LEFT);
   rotwheel(BACK, RIGHT);

   return returnValue;
}

bool robotDriver::right()
{
   bool returnValue = false;

   //Rotate wheels
   rotwheel(BACK, LEFT);
   rotwheel(FORWARD, RIGHT);

   return returnValue;
}

bool robotDriver::reverse()
{
   bool returnValue = false;

   //Rotate wheels
   rotwheel(BACK, LEFT);
   rotwheel(BACK, RIGHT);

   return returnValue;
}

void robotDriver::stop()
{
   //Rotate wheels
   rotwheel(STOP, LEFT);
   rotwheel(STOP, RIGHT);
}



void robotDriver::rotwheel(int dir, int in)
{
   //int pwm_r;


   //Slight left

   //Slight right

   if (in == RIGHT) {
      //pwm_r = PWM_scale * PWM_MAX_A/100;
      if (dir == BACK) {
         //_ctrl.set_data(ANALOG, PWMA, pwm_r);
         _ctrl.set_data(DIGITAL, INA1, 1);
         _ctrl.set_data(DIGITAL, INA2, 0);
      } else if (dir == STOP) {
         _ctrl.set_data(DIGITAL, INA1, 0);
         _ctrl.set_data(DIGITAL, INA2, 0);
      } else {
         //_ctrl.set_data(ANALOG, PWMA, pwm_r);
         _ctrl.set_data(DIGITAL, INA1, 0);
         _ctrl.set_data(DIGITAL, INA2, 1);
      }
   } else {
      //pwm_r = PWM_scale * PWM_MAX_B/100;
      if (dir == BACK) {
         //_ctrl.set_data(ANALOG, PWMB, pwm_r);
         _ctrl.set_data(DIGITAL, INB1, 0);
         _ctrl.set_data(DIGITAL, INB2, 1);
      } else if (dir == STOP) {
         _ctrl.set_data(DIGITAL, INB1, 0);
         _ctrl.set_data(DIGITAL, INB2, 0);
      } else {
         //_ctrl.set_data(ANALOG, PWMB, pwm_r);
         _ctrl.set_data(DIGITAL, INB1, 1);
         _ctrl.set_data(DIGITAL, INB2, 0);
      }
   }
}

void robotDriver::start()
{
   _thread_exit = false;

   std::thread t1(&robotDriver::move_thread, this);
   t1.detach();

   std::thread t2(&robotDriver::brake_thread, this);
   t2.detach();

}

void robotDriver::move_thread(robotDriver* ptr)
{
   while (ptr->_thread_exit == false) {
      ptr->move();
   }
}

void robotDriver::brake_thread(robotDriver* ptr)
{
   while (ptr->_thread_exit == false) {
      ptr->brake();
   }
}

void robotDriver::move()
{

//Test script
/*
   static double testTime = cv::getTickCount();
   if((cv::getTickCount()-testTime)/cv::getTickFrequency() >= 0.1) {
      testTime = cv::getTickCount();
      switch(_dir_actual) {
         case FORWARD:
            std::cout<<"FORWARD\n";
            break;
         case BACK:
            std::cout<<"BACK\n";
            break;
         case LEFT:
            std::cout<<"LEFT\n";
            break;
         case RIGHT:
            std::cout<<"RIGHT\n";
            break;
         case SLEFT:
            std::cout<<"SLEFT\n";
            break;
         case SRIGHT:
            std::cout<<"SRIGHT\n";
            break;
      }
   }
   */

//NON TEST SCRIPT
//This function is needed because when a user changes the direction, it takes a little bit of time
//to change direction, as the motors have to be slowed down, set to 0, and then sped back up
//to make sure there's no Back EMF and it's not hard on the motors.
//
//Once the motors are actually stopped, THEN the actual direction can be changed
   if((cv::getTickCount()-msdelay)/cv::getTickFrequency() >= 0.001) {
      msdelay=cv::getTickCount();
      switch (_dir_actual) {
         case FORWARD:
         case SLEFT:
         case SRIGHT:
            forward();
            break;
         case BACK:
            reverse();
            break;
         case LEFT:
            left();
            break;
         case RIGHT:
            right();
            break;
         default:
            stop();
            break;
      }
   }
}

void robotDriver::brake()
{
   if(_dir_actual==SLEFT) PWM_halfleft = 0.55;
   else if(_dir_actual == LEFT || _dir_actual == RIGHT) PWM_halfleft = 0.65;
   else PWM_halfleft = 1;

   if(_dir_actual==SRIGHT) PWM_halfright = 0.55;
   else if(_dir_actual == LEFT || _dir_actual == RIGHT) PWM_halfright = 0.65;
   else PWM_halfright=1;


   //If PWM scalar is higher than 0, then it needs to slow down
   if (_brakePrompt) {
      while (!_braked) {
         if (PWM_scale > 0) {
            if ((cv::getTickCount() - brakeTime) / cv::getTickFrequency() >= BRAKE_INT) {
               brakeTime = cv::getTickCount();
               PWM_scale -= BRAKE_DCL;
               if (PWM_scale <= 0) {
                  PWM_scale = 0;
                  _braked = true;
               }
               _ctrl.set_data(ANALOG, PWMA, PWM_halfleft * PWM_scale * PWM_MAX_A / 100);
               _ctrl.set_data(ANALOG, PWMB, PWM_halfright * PWM_scale * PWM_MAX_B / 100);
            }
         }
      }
   } else {
      //accelerate
      if ((PWM_scale < 100) && (cv::getTickCount() - accelTime) / cv::getTickFrequency() >= BRAKE_INT) {
         accelTime = cv::getTickCount();
         PWM_scale += BRAKE_ACL;
         if (PWM_scale > 100) PWM_scale = 100;
         _ctrl.set_data(ANALOG, PWMA, PWM_halfleft * PWM_scale * PWM_MAX_A / 100);
         _ctrl.set_data(ANALOG, PWMB, PWM_halfright * PWM_scale * PWM_MAX_B / 100);
      }
   }

   //This will get executed when _braked is on
   if (_braked) {
      //usleep(50000);
      _dir_actual = _dir;
      _brakePrompt = false;
      _braked = false;
   }
}
