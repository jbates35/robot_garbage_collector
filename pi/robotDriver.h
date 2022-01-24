#pragma once

#include "CControl_Pi.h"

#include <thread>
#include <mutex>
#include <vector>
#include <unistd.h>

enum
{
   FAIL = 0,
   SUCCESS,
   BUSY
};
enum
{
   STOP = 0,
   FORWARD,
   BACK,
   LEFT,
   RIGHT,
   SLEFT,
   SRIGHT,
   HALF
};

#define PWM_MAX_A 57
#define PWM_MAX_B 57

#define BRAKE_INT 0.01
#define BRAKE_ACL 4
#define BRAKE_DCL 4

/**
*    @brief Robot motor control script
*    @details Implements a means of controlling the motors, therefore automatically speeding them up or not if need be
*    @author Jimmy Bates
*    @date April 26 2021
*/
class robotDriver
{
public:
   robotDriver();
   ~robotDriver();

	/**
	 * @brief Set direction of the motors
	 * @return Confirmation of direction set
	 * **/
     int setDir(int dir);
       /**
     * @brief Getter for the direction of motors
     * @return Direction of motors
     * **/
     int getDir();

private:
   CControl_Pi _ctrl; ///< CControl object used in CBase4618

   int PWM_scale;
   int PWM_busy;
   float PWM_halfright, PWM_halfleft;

   /*These are actually for the process of changing dir
   If _dir gets set by main, turn _brakePrompt on
   _dir_prev might be pointless*/
   int _dir, _dir_prev;

   /*If direction changes, brakePrompt must get turned on
   Direction can only actually change if _braked = true
   We can accomplish this by making _dir_actual = _dir*/
   bool _brakePrompt, _braked;

   /*There is also a thread that takes care of direction, basically it takes care of constantly updating
   the PI to move direction based on _dir_actual*/
   int _dir_actual;

   double accelTime, brakeTime;
   double msdelay;

   //CTRL pi inputs/outputs
   std::vector<bool> _ready;
   std::vector<int> _input;
   std::vector<int> _inputtype;
   std::vector<int> _output;
   std::vector<int> _outputtype;
   std::vector<int> _button;


   bool forward();
   bool left();
   bool right();
   bool reverse();
   void rotwheel(int dir, int in);
   double goTime;
   bool _thread_exit;
   void start();
   static void move_thread(robotDriver* ptr);
   static void brake_thread(robotDriver* ptr);
   void move();
   void stop();
   void brake();
};

