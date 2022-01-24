#Robot Garbage Collector
##A robot that uses computer vision to collect marbles

Video of project found here: 
* (autonomous mode)[https://www.youtube.com/watch?v=0-Qhag0Sz4Y]
* (manual mode)[https://www.youtube.com/watch?v=ChK4AZdKRys]

This project is divided into two parts:
* PC
* Pi

Most of the programming was done on the Pi. The main file is robot.cpp/robot.h. In order to simplify the project, I created different classes for the components of the robot, such as the camera, motors, servos. There is a networking component to this project as well.

The script for the PC is simply to stream the robot's camera feed, while being able to either set it to autonomous mode or manual mode and control it from there.