#include "stdafx.h"
#include "piServer.h"

piServer::piServer()
{
   _matsize = cv::Size(MATW, MATH);
   _canvas = cv::Mat::zeros(_matsize, CV_8UC3);

   initVars();
   start();
   /*
   std::cout << "\nCommands:";
   std::cout << "\n(O) Turn system on or off";
   std::cout << "\n(G) Camera shot";
   std::cout << "\n(L) Send to left bin";
   std::cout << "\n(R) Send to right bin";
   std::cout << "\n(1) Get bin 1 count";
   std::cout << "\n(2) Get bin 2 count\n\n";
   */
}

piServer::~piServer()
{
}

void piServer::update()
{

   //get user command if any
   getCmd();

   //send user command to server if any
   sendCmd();
}

void piServer::draw()
{  
   //Reset canvas
   _canvas = cv::Mat::zeros(_matsize, CV_8UC3);

   //Transfer last image to frame
   if (!_im.empty()) _canvas = _im.clone();

   //Put text and draw frame
   //drawTextFrame();

   //Show image and get keystroke
   cv::Mat im_show;
   cv::resize(_canvas, im_show, cv::Size(), 2, 2);

   cv::imshow("CAM", im_show);
   keyPress = cv::waitKey(100);
}

void piServer::start()
{
   _thread_exit = false;

   std::thread t1(&piServer::userInput_thread, this);
   t1.detach();

}

void piServer::getCmd()
{
   //Get return from pi
   //If return, turn serverbusy off and process
   _pi1.rx_str(rxStr);
   if (rxStr != "") {

      if (rxStr.at(0) == 'a') {

         //Parse data 
         int word = 0;
         std::string parse;
         std::stringstream decodess(rxStr);
         std::vector<int> container;

         while (std::getline(decodess, parse, ' ')) {
            if (word >= 1 && word <= 2) {
               container.push_back(stoi(parse));
            }
            word++;
         }

         //Execute action based on what container is
         decisionTree(container);

      }

      rxStr = "";
   }
   
    cv::Mat imtmp;

    _pi2.tx_str("im");

    //Get return from camera
    if (_pi2.rx_im(imtmp)) {
        if (imtmp.empty() == false) {
        _im = imtmp.clone();
        _matsize = cv::Size(_im.cols, _im.rows);
        }
    }


 }


void piServer::sendCmd()
{
   //Send client data to Pi
   //If command == 1, 'G', if command == 2, 'S'
   if (command >= 1) {
      command = 0;

      //Send command (cmdStr)
      _pi1.tx_str(cmdStr);

      //Blank cmdStr
      cmdStr = "";
   }
}

void piServer::userInput_thread(piServer* ptr)
{
   while (ptr->_thread_exit == false) {
      ptr->userInput();
   }
}

void piServer::decisionTree(std::vector<int> container)
{
   switch (container[0]) {
   case 0:
      sysOn = container[1];
      break;
   case 1:
      msgflag = true;
      msgtimer = cv::getTickCount();
      if (container[1] == 0) msgindex = 0;
      else msgindex = 1;
      break;
   case 2:
      bin1 = container[1];
      break;
   case 3:
      bin2 = container[1];
      break;

   case 9:
      msgflag = true;
      msgtimer = cv::getTickCount();
      msgindex = 2;
   }
}

void piServer::drawTextFrame()
{
}

void piServer::userInput()
{
   std::string str = "";

   //Move robot
   if (debKey('a') || debKey('A')) {
       //Move robot left
       str = "S 4 3 '\\n'";
   }
   if (debKey('s') || debKey('S')) {
       //Move robot back
       str = "S 4 2 '\\n'";
   }
   if (debKey('d') || debKey('D')) {
       //Move robot right
       str = "S 4 4 '\\n'";
   }
   if (debKey('w') || debKey('W')) {
       //Move robot forward
       str = "S 4 1 '\\n'";
   }
   if (debKey('c') || debKey('C')) {
       //Stop robot
       str = "S 4 0 '\\n'";
   }


   if (debKey('k') || debKey('K')) {
      //Open garbage collector
      str = "S 5 1 '\\n'";
   }

   if (debKey('l') || debKey('L')) {
      //Close garbage collector
      str = "S 5 0 '\\n'";
   }

   if (debKey('o') || debKey('O')) {
      //Set system on
      str = "S 0 1 '\\n'";
   }


   if (debKey('1')) { // Set
       str = "S 6 1 \n";
   }

   if (debKey('2')) { // Run
       str = "S 6 2 \n";
   }

   if (debKey('3')) { // Control
       str = "S 6 3 \n";
   }

   if (debKey('0')) { // Turn off, or return to nostate
       str = "S 6 0 \n";
   }

   if (debKey('g') || debKey('G')) { //GRABIT
      str = "S 7\n";
   }

   if (str != "") {
      if (str.at(0) == 'G') {
         command = 1;
      }
      if (str.at(0) == 'S') {
         command = 2;
      }
      cmdStr = str;
   }
}

void piServer::initVars()
{
   bin1 = 0;
   bin2 = 0;
   keyRelease = 1;
   keyPressed = 0;
   keyTime = cv::getTickCount();
   command = 0;
   camcommand = false;
   serverBusy = false;
   _thread_exit = false;
   cmdStr = "";
   camStr = "";
   rxStr = "";
   camOn = false;
   sysOn = false;
   imageBusy = false;
   bordCol = { SYSOFFCOL, SYSONCOL };
   miscstrings = {
      "Marble sent to the left!",
      "Marble sent to the right!",
      "Command could not be executed."
   };
   msgflag = false;
   msgindex = 0;
   msgtimer = cv::getTickCount();
   _im = cv::Mat::zeros(cv::Size(640,480), CV_8UC3);

}

//Function to debounce keyboard
bool piServer::debKey(char key)
{
   bool returnval = false;


   if (!keyPressed) {
      if (keyPress == key) {
         keyTime = cv::getTickCount();
         keyPressed = true;
      }
   } else {
      if ((cv::getTickCount() - keyTime) / cv::getTickFrequency() >= 0.5 && keyPress == key) {
         returnval = true;
         keyRelease = false;
         keyPressed = false;
         lastKey = key;
      }
   }

   return returnval;
}