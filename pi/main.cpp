#include "robot.h"
#include <vector>
#include <X11/Xlib.h>

//MAIN FILE
//All this really does is just initialize PiGPIO and make sure it's up and running
//And once that is done, generate the robot object which is the heart of this project
int main()
{

    XInitThreads();
    
    if(gpioInitialise() < 0) {
        return 1;
    }  else {

        robot _robot;
        _robot.run();

    }

    gpioTerminate();

}
