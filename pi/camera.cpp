#include "camera.h"

std::mutex imgmut;

camera::camera()
{
   init_vars();
   start();

   //Open cam 
   cap.open(0);
   cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
   cv::waitKey(100);
}

camera::~camera()
{ 
}

//Essentially catch a keystroke on a rising edge instead of just being held down
//If you press "k" you don't want it to be interpretted as "kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk...."
bool camera::debkey(char key)
{
   bool returnval = false;

   if (keyRelease) {
      if (!keyPressed) {
         if (charkey == key) {
            keyTime = cv::getTickCount();
            keyPressed = true;
         }
      } else {
         if ((cv::getTickCount() - keyTime) / cv::getTickFrequency() >= 0.001 && charkey == key) {
            returnval = true;
            keyRelease = false;
            keyPressed = false;
            lastkey = key;
         }
      }
   } else {
      if ((lastkey == key) && (charkey != key) && !keyPressed) {
         keyRelease = true;
      }
   }
   return returnval;
 
}

//Initialization vars
void camera::init_vars()
{
   _im = cv::Mat::zeros(cv::Size(10, 10), CV_8UC3);
   _im_stored = cv::Mat::zeros(cv::Size(10, 10), CV_8UC3);
   _im_test = cv::Mat::zeros(cv::Size(10, 10), CV_8UC3);
   _im_process = cv::Mat::zeros(cv::Size(10, 10), CV_8UC3);
   _cont_center = cv::Point(0, 0);
   _ball_close = false;
   _dir_change = C_STOP;
   _thread_exit = false;
   _ball_state = false;
   charkey = ' ';
   lastkey = ' ';
   keyTime = cv::getTickCount();
   keyRelease = true;
   keyPressed = false;
   returnHome = false;
   _modeState = NOSTATE;
   _infoss.str("");
   _ROI = ROI_RECT;
   _ball_loc = 0;
   _garbagecount = 0;

   _controlss.str("");
   _controlvec.clear();

   procTime = cv::getTickCount();
   procRecord = 0;

   _jam = false;


}

void camera::imageGet()
{
   //Get video camera image
   cap.read(_im);

   // This will be used to get ROI and what not
   //ROI from top of servo area, and width of areas as well

   if(!_im.empty()) {
      _im_process = _im(_ROI).clone();
      cv::resize(_im_process,_im_process, cv::Size(), 0.5, 0.5);
      _size = cv::Size(_im_process.cols, _im_process.rows);
      
   }
}


void camera::imageProcess()
{

   //Mats used
   cv::Mat im_hsv;
   cv::Rect cont;


   //Flags for homebase and ball
   // [1] = home base
   // [0] = ball
   std::vector<bool> cflag = { false, false };

   //Convert to HSV and find range
   cv::cvtColor(_im_process, im_hsv, BGR2HSV);

   //In range mats and HSV color boundaries, also contour variables
   std::vector<cv::Mat> im_range = { im_hsv.clone(), im_hsv.clone() };
   std::vector<cv::Scalar> im_hsv_min = { HSVMIN_DICE, HSVMIN_BASE };
   std::vector<cv::Scalar> im_hsv_max = { HSVMAX_DICE, HSVMAX_BASE };
   std::vector<cv::Vec4i> hierarchy;
   std::vector<std::vector<cv::Point>> contours, contoursMax;

   //Create mat of objects in HSV range
   cv::inRange(im_hsv, im_hsv_min[returnHome], im_hsv_max[returnHome], im_range[returnHome]);

   //erode and dilate with larger element so make sure object is nicely visible
   cv::Mat erodeElement = getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
   cv::Mat dilateElement = getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
   cv::erode(im_range[returnHome], im_range[returnHome], erodeElement);
   cv::erode(im_range[returnHome], im_range[returnHome], erodeElement);
   cv::dilate(im_range[returnHome], im_range[returnHome], dilateElement);

   //Detect contour
   cv::findContours(im_range[returnHome], contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

   //Push back the max contour to do image processing on
   int maxcont = getMaxAreaContour(contours);

   if (maxcont != -1) {
      //Update contours with object
      contoursMax.push_back(contours[maxcont]);
      cv::drawContours(_im_process, contoursMax, -1, cv::Scalar(100, 0, 255), 1, cv::LINE_AA);

      //Find center
      _cont_center = cv::Point(_cont_rect.x + _cont_rect.width / 2, _cont_rect.y + _cont_rect.height / 2);

      //Let program know that there is a ball found and a rect updated
      _ball_state = true;
   } else {
      _ball_state = false;
      _cont_center = cv::Point(-1,-1);
   }

}

//update class variable to tell main script where in the shot the ball is
void camera::imageDirection()
{
   static double testtime = cv::getTickCount();

   //Setting boundaries of the frame
   cv::Rect _OPENAREA = OUT_RECT;
   cv::Rect _SERVOAREA = IN_RECT;
   cv::Rect _OPENAREAHALF = OUT_HALF_RECT;
   cv::Rect _INROIRECT = SHIFTED_HALF_RECT;

   int centerleft = _size.width / 2 - 3;
   int centerright = _size.width / 2 + 3;

   int farleft = _size.width / 2 - 10;
   int farright = _size.width / 2 + 10;

   int bottomboundary = _size.height - 10; 

   //If ball found, set ball location varaible where ball is found
   if (_ball_state) {

      if (_cont_center.x <= farleft) _ball_loc = C_LEFT;
      if (_cont_center.x < centerleft && _cont_center.x > farleft) _ball_loc = C_SLEFT;

      if (_cont_center.x > centerright && _cont_center.x < farright) _ball_loc = C_SRIGHT;
      if (_cont_center.x >= farright) _ball_loc = C_RIGHT;

      if (_cont_center.x <= centerright && _cont_center.x >= centerleft) _ball_loc = C_CENTRE;

   } else {

      _ball_loc = 0; //NOT FOUND

   }

   //If ball has passed through the into the garbage collector area
   if (_INROIRECT.contains(_cont_center)) _passUpperBound = true;
   else _passUpperBound = false;

}

void camera::imageTest()
{
   if (_modeState == SET) {

      //Goal of this function is to draw a straight line down the middle to align the camera on the pi
      int x = 30;
      int maxx = _im.cols / x;
      if (maxx != 0) {
         for (int i = 0; i < maxx; i++) {
            cv::line(_im, cv::Point(x, 0), cv::Point(x, _im.rows), CV_RGB(160, 160, 160), 1);
            x += 30;
         }
      }

      cv::line(_im, cv::Point(_im.cols / 2, 0), cv::Point(_im.cols / 2, _im.rows), CV_RGB(255, 255, 0), 2);
   }

   //Draw bounding boxes
   cv::rectangle(_im, _ROI, CV_RGB(0, 255, 255), 2, cv::LINE_AA);
   cv::rectangle(_im, IN_RECT, CV_RGB(180, 0, 255), 1, cv::LINE_AA);
   cv::rectangle(_im, OUT_RECT, CV_RGB(121, 121, 255), 1, cv::LINE_AA);
}

int camera::getMaxAreaContour(std::vector<std::vector<cv::Point>> contours)
{
   //Credit goes to stackoverflow for this one - https://stackoverflow.com/questions/46187563/finding-largest-contours-c
   //Find biggest object in contour vector and return it

   double maxArea = 60;
   if (returnHome) maxArea = 1000;
   int maxAreaContID = -1;

   for (int i = 0; i < contours.size(); i++) {
      double newArea = cv::contourArea(contours.at(i));
      if (newArea >= maxArea) {
         _cont_area = newArea;
         maxArea = newArea;
         maxAreaContID = i;
         _cont_rect = cv::boundingRect(contours.at(i));
      }
   }
   return maxAreaContID;
}

//This is the information string that is written to the camera feed
void camera::writeString()
{

   _infoss.str("");
   _infovec.clear();

   std::string ballfound = "LOCATING";
   if (_ball_state) ballfound = "FOUND";

   //Ball location
   std::string ballloc = "";
   switch (_ball_loc) {
   case C_LEFT:
      ballloc = "<<LEFT";
      break;
   case C_RIGHT:
      ballloc = "RIGHT>>";
      break;
   case C_CENTRE:
      ballloc = "^^CENTRE^^";
      break;
   case C_SLEFT:
      ballloc = "<SLIGHTLY LEFT";
      break;
   case C_SRIGHT:
      ballloc = "SLIGHTLY RIGHT>";
      break;
   default:
      break;
   }

   //What direction the robot needs to move
   std::string dir = "";
   switch (_dir_change) {
   case C_LEFT:
      dir = "<<LEFT";
      break;
   case C_RIGHT:
      dir = "RIGHT>>";
      break;
   case C_FORWARD:
      dir = "^^FORWARD^^";
      break;
   case C_BACK:
      dir = "vvBACKvv";
      break;
   case C_SLEFT:
      dir = "SLIGHT LEFT";
      break;
   case C_SRIGHT:
      dir = "SLIGHT RIGHT";
      break;
   default:
      dir = "STOPPED";
      break;
   }

   //Processing time of opencv stream
   _infoss << std::setprecision(2) << "Processing time: " << procRecord << "s";

   _infovec.push_back(_infoss.str());
   _infoss.str("");

   //Print coordinates of ball found
   if (_ball_state) {
      _infoss << "Contour area: " << _cont_area;
      _infovec.push_back(_infoss.str());
      _infoss.str("");

      _infoss << "Contour center: (" << _cont_center.x << ", " << _cont_center.y << ")";
      _infovec.push_back(_infoss.str());
      _infoss.str("");
   }
   //What direction the robot actually is moving
   _infoss << "Robot direction: " << dir;
   _infovec.push_back(_infoss.str());
   _infoss.str("");

   //How many marbles has the robot collected
   _infoss << "Garbage count: " << _garbagecount;
   _infovec.push_back(_infoss.str());
   _infoss.str("");

   //Has the ball been collected?
   _infoss << "Returning home? ";
   if (returnHome) _infoss << "Yes";
   else _infoss << "No";
   _infovec.push_back(_infoss.str());
   _infoss.str("");

   //If robot is opening or closing servo
   if (_passUpperBound) _infoss << "Robot state: COLLECTING";
   else _infoss << "Robot state: " << ballfound;
   _infovec.push_back(_infoss.str());
   _infoss.str("");

   if (_ball_state) _infoss << "Ball location: " << ballloc;
   _infovec.push_back(_infoss.str());
   _infoss.str("");

   //WRITE THIS IF ROBOT IS JAMMED
   if(_jam) _infoss << "JAMMED";
   _infovec.push_back(_infoss.str());
   _infoss.str("");

   _controlss.str("");
   _controlvec.clear();

   //Control variables in right side of screen for manual control
   if (_modeState == CONTROL) {
      _controlvec.push_back("w: forward");
      _controlvec.push_back("a: turn left");
      _controlvec.push_back("s: back");
      _controlvec.push_back("d: turn right");
      _controlvec.push_back("q: slight left");
      _controlvec.push_back("e: slight left");
      _controlvec.push_back("c: stop");
      _controlvec.push_back("");
      _controlvec.push_back("a: servo open");
      _controlvec.push_back("a: servo close");
      _controlvec.push_back("");
      _controlvec.push_back("0: back to nostate");
   }

   //Print options at start of stream
   if (_modeState == NOSTATE) {
      _controlvec.push_back("1: setup/calibrate");
      _controlvec.push_back("2: autonomous run");
      _controlvec.push_back("3: manual control");
      _controlvec.push_back("0: exit");
   }

   //Print this if robot is in autonomous mode
   if (_modeState == RUN || _modeState == SET) {
      _controlvec.push_back("0: back to nostate");
   }
}

void camera::drawFrame(cv::Mat& im)
{
    //Draw frame of reference on camera feed
   cv::Scalar frameclr = cv::Scalar(255, 255, 255);

   switch (_modeState) {
   case SET:      frameclr = cv::Scalar(0, 0, 255);  break;
   case RUN:      frameclr = cv::Scalar(0, 255, 0);  break;
   case CONTROL:  frameclr = cv::Scalar(255, 0, 0);  break;
   default: break;
   }

   cv::Rect framerect = cv::Rect(0, 0, im.cols, im.rows);
   cv::rectangle(im, framerect, frameclr, 3, cv::LINE_AA);
}

void camera::drawText(cv::Mat& im)
{
   cv::Scalar textclr = cv::Scalar(255, 255, 255);
   std::string modestr = "NO STATE";

   switch (_modeState) {
   case SET:      textclr = cv::Scalar(0, 0, 255); modestr = "SET";      break;
   case RUN:      textclr = cv::Scalar(0, 255, 0); modestr = "RUN";      break;
   case CONTROL:  textclr = cv::Scalar(255, 0, 0); modestr = "CONTROL";  break;
   default: break;
   }

   writeString();
   int j = 15;
   if (!_infovec.empty()) {
      for (auto i : _infovec) {
         cv::putText(im, i, cv::Point(10, j), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1);
         j += 15;
      }
   }

   j = 15;
   if (!_controlvec.empty()) {
      for (auto i : _controlvec) {
         cv::putText(im, i, cv::Point(im.cols - 150, j), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 170, 105), 1);
         j += 15;
      }
   }

   std::string imstring = "Command: " + modestr;
   cv::putText(im, imstring, cv::Point(15, im.cols - 5), cv::FONT_HERSHEY_DUPLEX, 0.4, textclr, 1);
}

void camera::update()
{
   procTime = cv::getTickCount();

   imageGet();

   if (!_im.empty()) {
      imageProcess();
      imageDirection();

      if (_modeState == SET || _modeState == CONTROL) {
         imageTest();
      }
   }

   writeString();

   procRecord = (cv::getTickCount() - procTime) / cv::getTickFrequency();
}

//CAMERA DRAW FUNCTIONS
void camera::draw()
{
   if (!_im.empty()) {
      drawFrame(_im);
      drawText(_im);
      imgmut.lock();
      _im_stored = _im.clone();
      imgmut.unlock();
   }
}

//CAMERA RUN METHOD
void camera::run()
{
   update();
   draw();
}


void camera::getMat(cv::Mat& im)
{
   imgmut.lock();
   im = _im_stored;
   imgmut.unlock();
}

void camera::run_thread(camera* ptr)
{
   while (ptr->_thread_exit == false) {
      ptr->run();
   }
}

void camera::start()
{
   std::thread t1(&camera::run_thread, this);
   t1.detach();

   std::thread t2(&camera::jam_thread, this);
   t2.detach();
}

void camera::jam_thread(camera* ptr) 
{
   while (ptr->_thread_exit == false) {
      ptr->jam();
   }
}

void camera::jam()
{   
   //Check every 0.2s
   if ((cv::getTickCount() - jamtimer) / cv::getTickFrequency() >= 0.4) {

      jamtimer = cv::getTickCount();

      //If contour has changed, or contour doesn't exist, reset jambool
      if (_cont_center_prev != _cont_center || _cont_center == cv::Point(-1, -1)) {
         _cont_center_prev = _cont_center;

         //Reset vector
         jambool.clear();

      } else {
         if (jambool.size() < 5) jambool.push_back(true);
      }
   }

   //If contour centers stays stuck for 2s, turn on jam
   if (jambool.size() >= 5) _jam = true;
   else _jam = false;
}