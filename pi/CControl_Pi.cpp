/**
*    @brief Serial communication w MSP432
*    @details This class is used to create an object that can init, set and get via Serial with the MSP432
*    @author Jimmy Bates
*    @version 0.1
*    @date Jan 25 2021
*/

#include "CControl_Pi.h"

CControl_Pi::CControl_Pi()
{
   _ready = false;
   button.assign((PINCOUNT + 1), 0);
   released.assign((PINCOUNT + 1), 1);
   contflag.assign((PINCOUNT + 1), 0);
   gb_timestart.assign((PINCOUNT + 1), 0);
   updateFlag.assign((PINCOUNT + 1), 0);
   rxData.assign((PINCOUNT + 1), 0);
}

CControl_Pi::~CControl_Pi()
{ 
}


////////////////////////////////////////////////////////
// init_com() - Initializes the serial comm
// arguments - comport - port of com
// return value - None allowed
////////////////////////////////////////////////////////
void CControl_Pi::init_com(std::vector<int> input, std::vector<int> inputtype, std::vector<int> button, std::vector<int> output, std::vector<int> outputtype)
{   
    /*
    _input = input;
    _inputtype = inputtype;

        //Cycle through and set digital inputs, and SPI, and servo
        for(int i=0; i<input.size(); i++) {
            if(inputtype[i]==DIGITAL) {
                gpioSetMode(input[i], PI_INPUT);
            } else if(inputtype[i]==ANALOG) {

            } else {

            } //end if-else tree
        } // end for

        // cycle through and set buttons
        for(auto i : button) {
            gpioSetMode(i, PI_INPUT);
        } // end for (getbutton)
*/
        // Cycle through and set outputs
        for(int i=0; i<output.size(); i++) {
            if(inputtype[i]==DIGITAL || inputtype[i]==SERVO) {
                gpioSetMode(i, PI_OUTPUT);
            } // end if
        } // end for (output)       

        //start();
}   // end init_com()

//Script responsible for polling all the inputs and keeping it in a thread that continuously loops
void CControl_Pi::start()
{
   _thread_exit = false;

   std::thread t1(&CControl_Pi::poll_inputs_thread, this);
   t1.detach();
}

void CControl_Pi::poll_inputs_thread(CControl_Pi* ptr)
{
   while (ptr->_thread_exit == false) {
      ptr->poll_inputs();
   }
}

bool CControl_Pi::poll_inputs()
{
   //Poll through inputs - if update flag is turned off, check input, set update flag
   for (int i = 0; i < _input.size(); i++) {
      get_data(_inputtype[i], _input[i], rxData[_input[i]]);
   }

   _ready = true;
}
//End polling script */



////////////////////////////////////////////////////////
// get_data() - Retrieves data (1/0) from serial
// arguments - type, channel, &result
// return value - true, false
////////////////////////////////////////////////////////
bool CControl_Pi::get_data(int type, int channel, int& result)
{

    // Return value var
    bool returnValue = false;

    //SPI Read values
    int read_val;
    unsigned char inBuf[3];
    char cmd[] = { 1, (128+16*channel), 0 }; // 128 + 16 * channel

    // Change what's happening based on what type
    switch(type) {

        case DIGITAL:
            result = gpioRead(channel);
            break;

        //Take care of SPI
        case ANALOG:
            handle = spiOpen(0, 200000, 3);
            spiXfer (handle, cmd, (char*) inBuf, 3);
            read_val = ((inBuf[1] & 3) << 8) | inBuf[2];
            result=read_val;
            spiClose(handle);
            
        break;

        case SERVO:
        break;
    }

    returnValue=true;
    return returnValue;

}   // end get_data()

////////////////////////////////////////////////////////
// set_data() - Writes data (1/0) to serial
// arguments - type, channel, &result
// return value - true, false
////////////////////////////////////////////////////////
bool CControl_Pi::set_data(int type, int channel, int val)
{

    // Return value var
    bool returnValue = false;

    // Change what's happening based on what type
    switch(type) {

        case DIGITAL:
            gpioWrite(channel, val);
            returnValue = true;
            break;

        case ANALOG:
           gpioPWM(channel, val*255/100);
           returnValue = true;
        break;

        case SERVO:
            gpioServo(channel, 500+(2000*val)/180);
            rxData[channel] = val;
        break;

    } // case type

    returnValue=true;
    return returnValue;

}   // end set_data()

////////////////////////////////////////////////////////
// ADCCalc() - Converts analogread -> ADC
// arguments - ADCinput from the analogread
// return value - %value of ADC
////////////////////////////////////////////////////////
int CControl_Pi::ADCCalc(int channel)
{
    //return channel * 100 / 1024;
    return rxData[channel] * 100 / 1024;
}  // end ADC Calc


////////////////////////////////////////////////////////
// get_button() - Returns value of a button press
// Needs debounce (.25s timeout)
//
// arguments - none
// return - true if button was pressed after 0.1s
////////////////////////////////////////////////////////
bool CControl_Pi::get_button(int channel, bool conttype)
{

   bool returnValue = false;
   int buttonResult = 1;

   //If button hasn't been pressed, see if it's turned on and update flag
   if (released[channel]) {
      if (!button[channel]) {

         get_data(DIGITAL, channel, buttonResult);
         button[channel] = !buttonResult;
         gb_timestart[channel] = cv::getTickCount();

      } else {

         //If button has been pressed, wait 10ms, return true if button is still pressed
         if ((cv::getTickCount() - gb_timestart[channel]) / cv::getTickFrequency() >= DEBOUNCE) {

            get_data(DIGITAL, channel, buttonResult);
            if (buttonResult == 0) {
               returnValue = true;
            }

            //Reset the channel flag
            button[channel] = OFF;

            //Turn release button off
            released[channel] = OFF;

         }
      }
      } else {

      //If button hasn't been released, check if released (1), and match results
      get_data(DIGITAL, channel, buttonResult);
      released[channel] = buttonResult;

   }
   
   //If button is supposed to be continuous and if button is still on
   //Then after 1s, reset timer, and turn contflag on
   if (conttype == LATCH && !released[channel]) {
      if ((cv::getTickCount() - gb_timestart[channel]) / cv::getTickFrequency() >= TIMEOUT) {
         gb_timestart[channel] = cv::getTickCount();
         contflag[channel] = ON;
      }
   } else {
      //Button has been released, so we must turn this flag off
      contflag[channel] = OFF;
   }

   //Now the button's been pressed for 0.5 second, so now we have to return "true" every 0.125s
   if (contflag[channel] && (cv::getTickCount() - gb_timestart[channel]) / cv::getTickFrequency() >= BUTTONINT) {
      returnValue = true;
      gb_timestart[channel] = cv::getTickCount();
   }
   
   return returnValue;
}





