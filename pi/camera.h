#pragma once

//Windows
//#include <opencv2/opencv.hpp>

//Pi
#include </usr/include/opencv2/opencv.hpp>

#include <vector>
#include <thread>
#include <mutex>
#include <string> 
#include <sstream>


//COPY THIS OVER:
#define BGR2HSV 40 // Opencv code for BGR to HSV
#define BGR2GRAY 6 // opencv code for BGR to gray for contours

//Roi of when servo should open
#define OUT_RECT           cv::Rect(332,245,96,45)
#define OUT_HALF_RECT      cv::Rect(166,123,48,23)

//Roi of in servo area:
#define IN_RECT            cv::Rect(310,337,140,44)
#define IN_HALF_RECT       cv::Rect(155,168,70,22)

//Roi of processing area:
#define ROI_RECT           cv::Rect(274,0,212,290)
#define ROI_HALF_RECT      cv::Rect(145,0,90,145)

//Shifted ROI to monitor im_process
#define SHIFTED_OUT_RECT   cv::Rect(58,245,96,45)
#define SHIFTED_HALF_RECT  cv::Rect(27,123,48,23)

//Home base:
#define HSVMIN_BASE cv::Scalar(55, 75, 125) // HSV Min to range
#define HSVMAX_BASE cv::Scalar(90, 150, 255) // HSV Max to range

//Dice:
#define HSVMIN_DICE cv::Scalar(104 , 63 , 62) // HSV Min to range
#define HSVMAX_DICE cv::Scalar(129  , 193 , 255) // HSV Max to range

enum
{
   C_STOP = 0,
   C_FORWARD,
   C_BACK,
   C_LEFT,
   C_RIGHT,
   C_SLEFT,
   C_SRIGHT,
   C_CENTRE
};

enum
{
   NOSTATE=0,
   SET,
   RUN,
   CONTROL
};

class camera
{
public:
   /**
   * @brief Constructor for camera object
   **/
   camera();
   ~camera();
   
   /**
   * @brief Returns userinput from opencv (i.e. keyboard)
   * @param key Input pressed on keyboard to check
   * @return True if key is pressed
   **/
   bool debkey(char key);

   /**
   * @brief Returns value of whether dice is in servo area
   * @return True if dice is in lower rect (servo area)
   **/
   bool in_inarea() { return _passLowerBound; }

   /**
   * @brief Returns value of whether dice is just outside servo area (open if true)
   * @return True if ball is in upper rect
   **/
   bool in_outarea() { return _passUpperBound; }

   /**
   * @brief Once collected, set this false
   **/
   void reset_outarea() { _passUpperBound = false; }

   /**
   * @brief Gets the direction of ball in camera shot
   * @return FORWARD=1, BACK=2, LEFT=3, RIGHT=4, SLEFT=5, SRIGHT=6
   **/
   int ballloc() { return _ball_loc; }

   /**
   * @brief Get stored image from mat to network over
   * @return cv::Mat image returned
   **/
   void getMat(cv::Mat& im);

   /**
   * @brief Sets the mode of program and camera
   * @param mode Sets the mode to 0 (NOSTATE), 1 (SET), or 2 (RUN)
   * @return None
   **/
   void setMode(int mode) { _modeState = mode; }

   /**
   * @brief Sets the direction info cam has about the dir the robot is going in
   * @param dir Direction: STOP=0, FORWARD=1....
   * @return None
   * **/
   void setDir(int dir) { _dir_change = dir; }

   /**
   * @brief Returns the state of the mode
   * @return 0 (NOSTATE), 1 (SET), or 2 (RUN)
   **/
   int getMode() { return _modeState; }

   /**
   * @brief Sets whether robot is collecting or returning
   * @param rhome value that rhome should be
   **/
   void setRHome(bool rhome) { returnHome = rhome; }

   /**
   * @brief Gets whether robot is collecting or returning
   * @return True if returning home, false otherwise
   **/
   bool getRHome() { return returnHome; }

   /**
   * @brief If there is an image in the stored network image
   * @return True (image is present) or false (no image)
   **/
   bool empty() { return _im_stored.empty(); }

   /**
   * @brief Sets the amount of dice collected
   * @param garbage int of dice collected
   **/
   void setgbg(int garbage) { _garbagecount = garbage;  }

   /** 
    * @brief Function to see if the location is jammed
    * @return True if contour center has not moved for 2seconds
    * **/
   bool isjammed() { return _jam; }

private:
   cv::Mat _im;
   cv::Mat _im_stored; //networked _im
   cv::Mat _im_test;
   cv::Mat _im_process;
   cv::Size _size;
   cv::Point _cont_center;
   cv::Rect _cont_rect;
   cv::VideoCapture cap; ///< Camera object
   double _cont_area;
   std::stringstream _infoss;

   std::stringstream _controlss;
   std::vector<std::string>_controlvec;

   cv::Rect _ROI; // Area of interest   
   int _garbagecount;

   double procTime;
   double procRecord;

   char charkey, lastkey;
   bool keyRelease, keyPressed;
   double keyTime;
   int _ball_close;
   int _dir_change; // left, right, center
   bool _thread_exit;
   int _ball_state; // found or not
   int _ball_loc;
   bool _passLowerBound; //Whehter ball passed lower boundary or not
   bool _passUpperBound; //Whether dice has passed through the area to open
   int _modeState; // 1 = setup, 2 = run
   bool returnHome;
   std::vector<std::string> _infovec;

   void init_vars();
   void imageGet();
   void imageProcess();
   void imageDirection();
   void imageTest();
   int getMaxAreaContour(std::vector<std::vector<cv::Point>> contours);
   void writeString();
   void drawFrame(cv::Mat& im);
   void drawText(cv::Mat& im);
   void update();
   void draw();
   void run();
   static void run_thread(camera* ptr);
   void start();

   static void jam_thread(camera* ptr); //See if contour cneter hasn't moved *much* in the past second
   void jam();

   std::vector<bool> jambool;
   bool _jam;
   cv::Point _cont_center_prev;
   double jamtimer;

};

