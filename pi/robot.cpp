#include "robot.h"

Server _serv1; //Thread for intake and outtake of commands
Server _serv2; //Thread for streaming video image

std::mutex robMut;

robot::robot()
{
    //Set attributes for robot GPIO pins
   _input = { 0 };
   _inputtype = { 0 };
   _button = { 0 };
   _output = { 20, 21 };
   _outputtype = { DIGITAL, DIGITAL };

      //Initialize object with PiGPIO
   _pi.init_com(_input, _inputtype, _button, _output, _outputtype);

   _pi.set_data(DIGITAL, 20, 0); //LED1
   _pi.set_data(DIGITAL, 21, 0); //LED2

   initvars();

   _servo.init(SERVOPIN1, SERVOMAX1, SERVOMIN1, SCLOSE);

}

robot::~robot()
{
}

void robot::run()
{
   start();

   while(!_exit) {

    //Set LEDs to be turned on to signify state of robot
      if (_sel_prev == _selection) {
         if (_selection == RUN) {
            _pi.set_data(DIGITAL, 20, 1);
            _pi.set_data(DIGITAL, 21, 0);
         } else if (_selection == CONTROL) {
            _pi.set_data(DIGITAL, 20, 0);
            _pi.set_data(DIGITAL, 21, 1);
         } else {
            _pi.set_data(DIGITAL, 20, 0);
            _pi.set_data(DIGITAL, 21, 0);
         }
         _sel_prev = _selection;
      }

      //Resets garbagecount, possibly remove
      if(_selection==NOSTATE) _garbageCount = 0;

      //Send main image - AND PUT ANY CAM RELATED CODE IN HERE IF POSSIBLE
      if((cv::getTickCount()-progTimer)/cv::getTickFrequency() >= 0.0333) {
         _cam.setgbg(_garbageCount);
         _cam.setMode(_selection);
         progTimer = cv::getTickCount();
         if(_cam.empty()==false) _cam.getMat(networkIm);
      }

      cv::imshow("MAIN IM", networkIm);

      //send network image
      _serv2.set_txim(networkIm);

      //Waitkey
      _esckey = cv::waitKey(30);

      //User input
      if(_esckey) {
         switch(_esckey) {
            case 'q': case 'Q':   if(_selection==CONTROL) setdir(SLEFT);      break;
            case 'w': case 'W':   if(_selection==CONTROL) setdir(FORWARD);    break;
            case 'e': case 'E':   if(_selection==CONTROL) setdir(SRIGHT);     break;
            case 'a': case 'A':   if(_selection==CONTROL) setdir(LEFT);       break;
            case 's': case 'S':   if(_selection==CONTROL) setdir(BACK);       break;
            case 'd': case 'D':   if(_selection==CONTROL) setdir(RIGHT);      break;
            case 'c': case 'C':   if(_selection==CONTROL) setdir(STOP);       break;

            case 'k': case 'K':   if(_selection==CONTROL) _servo.open();      break;
            case 'l': case 'L':   if(_selection==CONTROL) _servo.close();     break;

            case 'g': case 'G':   if(_selection==CONTROL) {
                                    robMut.lock();
                                    grabitrequest=true;
                                    robMut.unlock();
                                    std::cout<<"\n\nG PRESSED";
                                    }
                                  break;

            case '1':             if(_selection==NOSTATE) _selection=SET;     break;
            case '2':             if(_selection==NOSTATE) _selection=RUN;     break;
            case '3':             if(_selection==NOSTATE) _selection=CONTROL; break;
            case '0':             _selection=NOSTATE; setdir(STOP);           break;

            case 'm': case 'M':   setdir(STOP); _exit=true;                   break;

            default: break;
         }//end script
      }
   }

   _serv1.stop();
   _serv2.stop();

}

//Initialize all threads
void robot::start()
{
   std::thread rt3(&robot::autonomous_thread, this);
   rt3.detach();

   std::thread rt5(&robot::serverStart1_thread, this);
   rt5.detach();

   std::thread rt6(&robot::serverStart2_thread, this);
   rt6.detach();

   std::thread rt7(&robot::serverCmds_thread, this);
   rt7.detach();

}

//Initialize important variables when reset or started
void robot::initvars()
{
   _exit = false;
   _selection = NOSTATE;
   _sel_prev = NOSTATE;
   cmds.clear();
   _garbageCount = 0;
   _in_outarea = false;
   _in_inarea = false;
   _relocate = false;
   found = false;
   _grabit = false;
   networkIm = cv::Mat::zeros(cv::Size(300,300), CV_8UC3);
   _esckey=1;
   progTimer = cv::getTickCount();
   _program_timer = cv::getTickCount();
   _found_timer = cv::getTickCount();
   grabitrequest=false;

}

//Set direction of robot motor object
void robot::setdir(int dir)
{
   while(_motors.getDir()!=dir) {
      _motors.setDir(dir);
   }
   _cam.setDir(dir);
}

void robot::autonomous_thread(robot* ptr)
{
   while (ptr->_exit == false) {
      ptr->autonomous();
   }
}

//MAIN RUN SCRIPT:

void robot::autonomous()
{

   //Non autonomous but if grabit requested:
   if(grabitrequest==true) {
      std::cout << "\n\nGRABBED IT REQUESTED";
      grabitrequest = false;
      grabit();
   }

   if(_selection==RUN) {

      if(_cam.in_outarea() == false) {
         _in_outarea = false;
         _in_inarea = false;


         //If no dice is found
         if(!found) {
            _found_timer = cv::getTickCount();
            found=false;
            setdir(RIGHT);
            if(_cam.ballloc() == C_RIGHT || _cam.ballloc() == C_SRIGHT || _cam.ballloc() == C_CENTRE || _cam.ballloc() == C_SLEFT || _cam.ballloc() == C_LEFT) {
               setdir(STOP);
               found=true;

               //Ball found
               std::cout << "\n\nFOUND";
               while((cv::getTickCount()-_found_timer)/cv::getTickFrequency()<=0.75) { }
               std::cout << "\n\nMOVING ON";

               //Move forward after finding ball
               if(_cam.ballloc()==C_RIGHT) setdir(SRIGHT);
               else if(_cam.ballloc()==C_LEFT) setdir (SLEFT);
               else setdir (FORWARD);
               while((cv::getTickCount()-_found_timer)/cv::getTickFrequency()<=1.25) { }
            }
         }

         //Robot has found dice, so drive accordingly
         if(found) {
            if(_cam.ballloc() == C_CENTRE) setdir(FORWARD);
            else if(_cam.ballloc() == C_LEFT) setdir(SLEFT);
            else if(_cam.ballloc() == C_RIGHT) setdir(SRIGHT);
            else if(_cam.ballloc() == 0) {
               found=false;
               _found_timer = cv::getTickCount();
            }
         }

      } else {

         //Dice is in outside box
         grabit();

         //If dice is gone, go back to main loop
         if(_cam.ballloc()==false) {
            found = false;
         }

         //Take back the garbagecount if it didn't enclose the garbage
         if(_cam.in_outarea()==true) _garbageCount--;
         if(_garbageCount >= BALLS_MAX) _cam.setRHome(true);
      }
   }
}

void robot::grabit()
{
   grabittime = cv::getTickCount();
   _grabit = true;

   std::cout << "\n\nABOUT TO GRABIT";

   while(_grabit) {

      //If return home = false
      if(_cam.getRHome()==false || _selection == CONTROL) {

         //Open servo and wait 1.5s
         setdir(STOP);
         _servo.open();
         grabittime = cv::getTickCount();
         while((cv::getTickCount()-grabittime)/cv::getTickFrequency()<=1.3) { }

         //Drive forward for .5s
         setdir(FORWARD);
         grabittime = cv::getTickCount();
         while((cv::getTickCount()-grabittime)/cv::getTickFrequency()<=0.4) { }


         //Close servo and wait 1.5s
         setdir(STOP);
         _servo.close();
         grabittime = cv::getTickCount();
         while((cv::getTickCount()-grabittime)/cv::getTickFrequency()<=1.5) { }
         grabittime = cv::getTickCount();


         if(_selection==RUN) {
            _garbageCount++;
         }

         //Exit loop
         _grabit=false;
         std::cout << "\n\nYA I GRABBED IT";

      } else { //if return home = true

         //Stop first
         setdir(STOP);
         while((cv::getTickCount()-grabittime)/cv::getTickFrequency()<=0.6) { }

         //Drive forward for .5s
         setdir(FORWARD);

         grabittime = cv::getTickCount();
         while((cv::getTickCount()-grabittime)/cv::getTickFrequency()<=0.3) { }


         //Open servo and wait 1.5s
         setdir(STOP);
         _servo.open();
         grabittime = cv::getTickCount();
         while((cv::getTickCount()-grabittime)/cv::getTickFrequency()<=2.0) { }


         //Back for 2s
         setdir(BACK);
         grabittime = cv::getTickCount();
         while((cv::getTickCount()-grabittime)/cv::getTickFrequency()<=2.0) { }


         //Selection = NOSTATE
         setdir(STOP);
         _selection=NOSTATE;
         _grabit=false;
         std::cout << "\n\nOH I RELEASED IT";

      }
   }
}

//END MAIN RUN SCRIPT

//OTHER THREAD INITIALIZATION
void robot::serverStart1_thread(robot* ptr)
{
   while (ptr->_exit == false) {
      ptr->serverStart1();
   }
}

void robot::serverStart1()
{
   _serv1.start(PORT1);
}

void robot::serverStart2_thread(robot* ptr)
{
   while (ptr->_exit == false) {
      ptr->serverStart2();
   }
}

void robot::serverStart2()
{
   _serv2.start(PORT2);
}

void robot::serverCmds_thread(robot* ptr)
{
   while (ptr->_exit == false) {
      ptr->serverCmds();
   }
}

void robot::serverCmds()
{
   std::stringstream txSS;
   _serv1.get_cmd(cmds);

   //parse through commands if they exist 1
   if (cmds.size() > 0) {
      for (auto i : cmds) {
         txSS << "a 9 9 \n";

         //Main commands:
         if (i.at(0) == 's' || i.at(0) == 'S') {


            std::cout << "\nReceived command " << i;

            switch (i.at(2)) {

               case '4':
                  if (_selection == CONTROL) {
                     switch (i.at(4)) {
                     case '0':
                        setdir(STOP); //Set direction of motors STOP
                        txSS.str("a 4 0 \n");
                        break;

                     case '1':
                        setdir(FORWARD); //SEt direction of motors FORWARD
                        txSS.str("a 4 1 \n");

                        break;

                     case '2':
                        setdir(BACK); //Set directino of motors BACKWARDS
                        txSS.str("a 4 2 \n");
                        break;

                     case '3':
                        setdir(LEFT); //Set direction of motors LEFT
                        txSS.str("a 4 3 \n");
                        break;

                     case '4':
                        setdir(RIGHT); //Set direction of motors RIGHT
                        txSS.str("a 4 4 \n");
                        break;

                     case '5':
                        setdir(SLEFT); //Set direction of motors SLIGHT LEFT
                        txSS.str("a 4 5 \n");
                        break;

                     case '6':
                        setdir(SRIGHT); //Set direction of motors SLIGHT RIGHT
                        txSS.str("a 4 6 \n");
                        break;

                     default:
                        setdir(STOP); //Set direction of motors STOP
                        txSS.str("a 4 0 \n");
                  } // end switch i.at(4)
               } // end if
               break;

            case '5':
               if (_selection == CONTROL && i.at(4) == '1') {
                  robMut.lock();
                  _servo.open();
                  robMut.unlock();
                  txSS.str("a 5 1 \n");
               } else if (_selection == CONTROL && i.at(4) == '0') {
                  robMut.lock();
                  _servo.close();
                  robMut.unlock();
                  txSS.str("a 5 0 \n");
               } //end if-else
               break;

            case '6':
               switch (i.at(4)) {
                  //Turn set on
               case '1':
                  robMut.lock();
                  _selection = SET;
                  robMut.unlock();
                  txSS.str("a 6 1 \n");
                  break;

                  //Turn run on
               case '2':
                  robMut.lock();
                  _selection = RUN;
                  robMut.unlock();
                  txSS.str("a 6 2 \n");
                  break;

                  //Turn control on
               case '3':
                  robMut.lock();
                  _selection = CONTROL;
                  robMut.unlock();
                  txSS.str("a 6 3 \n");
                  break;

                  //Turn off, or state=0
               case '0':
                  robMut.lock();
                  if(_selection==NOSTATE) _exit = true;
                  else {
                     _selection = NOSTATE;
                     setdir(STOP);
                  }
                  robMut.unlock();
                  txSS.str("a 6 0 \n");
                  break;

                  default: break;
               }
               break;

               case '7':
                  if(_selection==CONTROL) {
                     robMut.lock();
                     grabitrequest=true;
                     robMut.unlock();
                  }

               default: break;

            } // switch char at 2
         } // if i.at(0) = s

         //Send ACK back to server
         if(txSS.str() != "a 9 9 \n") {
            _serv1.send_string(txSS.str());
         } // end txss str send
      } //for, iterating through cmds
   } //if cmd has anything
}

