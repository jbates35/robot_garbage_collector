#pragma once

#include "stdafx.h"
#include "CBase4618.h"
#include "Client.h"

#include <string>
#include <vector>
#include <thread>
#include <mutex>

#define PI_IP "192.168.50.78"

#define MATW 640
#define MATH 480

#define PORT1 4618
#define PORT2 4619

#define SYSONCOL cv::Scalar(0, 255, 0)
#define SYSOFFCOL cv::Scalar(255,255,255)
#define BIN1COL cv::Scalar(100,190,255)
#define BIN2COL cv::Scalar(255,100,100)

#define SYSBORD 4

/**
*    @brief Connect to PI Server
*    @author Jimmy Bates
*    @date May 18 2021
*/
class piServer :
   public CBase4618
{

public:
   piServer();
   ~piServer();

   /**
   * @brief Continuously update project looking out for commands from pi
   * @return Nothing
   **/
   void update();

   /**
   * @brief Continuously draw on opencv canvas
   * @return Nothing
   **/
   void draw();

private:
   void start();
   void getCmd();
   void sendCmd();
   static void userInput_thread(piServer* ptr);
   void userInput();
   void initVars();
   bool debKey(char key);
   void decisionTree(std::vector<int> container);
   void drawTextFrame();
   void get_image();
   std::string cmdStr;
   std::string camStr;
   std::string rxStr;
   cv::Mat _im;
   cv::Size _matsize;
   Client _pi1 = Client(PORT1, PI_IP);
   Client _pi2 = Client(PORT2, PI_IP);
   std::vector<cv::Scalar> bordCol;
   std::vector<std::string> miscstrings;
   bool msgflag;
   int msgindex;
   double msgtimer;
   int bin1;
   int bin2;
   bool sysOn;
   bool keyRelease, keyPressed;
   double keyTime;
   char lastKey;
   bool serverBusy;
   bool imageBusy;
   int command;
   bool camcommand;
   bool _thread_exit;
   bool camOn;

};

