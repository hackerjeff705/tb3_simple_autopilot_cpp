# tb3_simple_autopilot_cpp
A simple autonomous robot in C/C++ that implements the following using the turtlebot3.
* MobileNetV1 that detects and determines the state of stoplights and stopsign. This runs in the raspberry pi 4b and sends the detected objects as a custom message to the computer.
* Pure Pursuit geometric controller along with a tool to create waypoints.
* Automatic emergency braking system using the lds-02 lidar on the tb3.

MobileNetV1 performance is approx 5 fps.

![alt text](https://github.com/adam-p/markdown-here/raw/master/src/common/images/icon48.png "Robot Flow Structure")

Feel free to install or copy the code for your own projects!

## Installation
* Create a package for your computer and raspberry pi4b.
* Download/clone the project
* Move the respective folders to their respective directories
* Build and run!
