# Autonomous Robot Project (in C++) with TurtleBot3

This project is an autonomous robot system that utilizes a TurtleBot3 platform, consisting of a Raspberry Pi 4 and a desktop computer, to perform various tasks such as object detection (specifically, recognizing stoplights and stop signs), waypoint-based navigation using the Pure Pursuit controller, and an automatic emergency braking system. The system is implemented using C/C++ and integrates multiple components to create a capable autonomous robot.

## Project Components

### 1. Raspberry Pi 4 Component
The Raspberry Pi 4 serves as the brain of the robot and is responsible for image processing and object detection. It runs a TensorFlow Lite model based on MobileNetV1 to detect and classify objects in real-time. The detected objects include stoplights and stop signs, which are critical for making navigation decisions.

#### Object Detection
The Raspberry Pi captures video frames from a camera, resizes them, and feeds them through the MobileNetV1 model. The model identifies objects in the frames, including their labels and bounding box coordinates. The detected objects are then processed and sent to the desktop component as custom messages.

### 2. Desktop Component
The desktop computer handles higher-level control and navigation tasks, including waypoint-based path planning and emergency braking. It communicates with the Raspberry Pi to receive object detection data and make navigation decisions.

#### Robot Controls (robot_controls.cpp)
- **Object Detection Integration**: This component subscribes to the custom object detection messages sent by the Raspberry Pi. It receives information about the detected objects, such as labels and positions in the frame.

- **Traffic Light State Determination**: Specifically, it identifies the state of traffic lights (either red or green) by analyzing the colors within the detected traffic light region. This information is crucial for obeying traffic rules.

- **Pure Pursuit Controller**: The Pure Pursuit algorithm is implemented here to guide the robot along a predefined path (waypoints). It calculates the appropriate velocity and steering angle based on the current position and orientation of the robot. The goal is to smoothly navigate the robot to reach the target waypoint.

- **Automatic Emergency Braking (AEB) System**: This component uses data from the LDS-02 LiDAR sensor to implement an automatic emergency braking system. It continuously monitors the robot's proximity to obstacles. If it detects a potential collision, it adjusts the robot's velocity to prevent accidents.

#### Waypoint Logger (waypoint_logger.py)
- **Waypoint Logging**: This Python script logs waypoints as the robot moves. It subscribes to the Odometry topic to track the robot's position, orientation, and speed. This information is recorded into a CSV file. Waypoints are essential for defining the robot's path for the Pure Pursuit controller.

## How the Robot Works

![alt text]( https://github.com/hackerjeff705/tb3_simple_autopilot_cpp/blob/main/burger_autonomous_stack.jpeg "Robot Flow Structure")

### Raspberry Pi 4 Component
This part of the system is responsible for real-time object detection. The Raspberry Pi captures video frames from the camera and uses the MobileNetV1 model to identify objects, particularly stoplights and stop signs. The detected objects, along with their labels and positions, are sent to the desktop component for decision-making.

### Desktop Component
The desktop computer is in charge of the robot's control and navigation. It integrates the following components:

- **Object Detection**: The object detection results received from the Raspberry Pi are used to determine the state of traffic lights (red or green) and the presence of stop signs. This information is vital for the robot's navigation decisions.

- **Pure Pursuit Controller**: The Pure Pursuit algorithm calculates the robot's velocity and steering angle based on its current position and orientation, as well as a predefined path of waypoints. This controller ensures that the robot follows the planned path while smoothly navigating curves and turns.

- **Automatic Emergency Braking (AEB) System**: Using data from the LiDAR sensor, the AEB system continuously monitors the distance between the robot and obstacles. If it detects an imminent collision, it takes control of the robot's velocity to prevent accidents.

- **Waypoint Logging**: Waypoints are logged as the robot moves. These waypoints represent the predefined path for the Pure Pursuit controller. The logging script subscribes to the Odometry topic to track the robot's position and orientation. These waypoints are crucial for guiding the robot along a specific route.

## Performance
- MobileNetV1 object detection on the Raspberry Pi 4b achieves a frame rate of approximately 5 frames per second (fps).
- The Pure Pursuit controller enables the robot to navigate smoothly and accurately along predefined waypoints.
- The AEB system enhances safety by detecting obstacles and taking appropriate actions.

Feel free to install or copy the code for your own projects!

## Usage
To use this project:
1. Deploy the provided object detection model on the Raspberry Pi 4b.
2. Configure your TurtleBot3 to run this code.
3. Create and define waypoints using the included tool.
4. Launch the robot's ROS nodes to start autonomous navigation.

## Dependencies
This project relies on the following dependencies:
- ROS (Robot Operating System)
- OpenCV
- TensorFlow Lite
- TurtleBot3 packages

## Installation
* Create a package for your computer and raspberry pi4b.
* Download/clone the project
* Move the respective folders to their respective directories
* Build and run!
