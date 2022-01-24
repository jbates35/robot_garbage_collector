#pragma once

/**
* @brief Robot class that runs the main robot garbage collection - script starts here
* @author Jimmy Bates
* @date May 11th 2021
* @version 0.1
* 
**/

#include "camera.h"
#include "servo.h"
#include "robotDriver.h"
#include "server.h"
#include "CControl_Pi.h"

#include <curses.h>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <sstream>
#include <mutex>

#define PORT1 4618
#define PORT2 4619

#define BALLS_MAX 4

class robot
{
public:
   robot();
   ~robot();

   /**
   * Run script
   **/
   void run();
   bool exit() { return _exit; }



private:

   void start();
   void initvars();
   void setdir(int dir);

   static void autonomous_thread(robot* ptr);
   void autonomous();

   void grabit();

   CControl_Pi _pi;
   camera _cam;
   robotDriver _motors;
   servo _servo;

   double _program_timer;
   double _found_timer;
   bool _in_outarea;
   bool _in_inarea;
   int _garbageCount;
   bool _relocate;
   bool found;
   bool _grabit;
   double grabittime;
   bool grabitrequest;


   double progTimer;

   cv::Mat networkIm;
   char _esckey;

   bool _exit;
   std::string _selectform;
   int _selection;
   int _sel_prev;

   std::vector<std::string> cmds;

   static void serverStart1_thread(robot* ptr);
   void serverStart1();
   static void serverStart2_thread(robot* ptr);
   void serverStart2();
   static void serverCmds_thread(robot* ptr);
   void serverCmds();

   //for ccontrol pi
   std::vector<int> _input;
   std::vector<int> _inputtype;
   std::vector<int> _button;
   std::vector<int> _output;
   std::vector<int> _outputtype;


};

