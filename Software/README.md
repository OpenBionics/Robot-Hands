# Introduction

<p style='text-align: justify;'>The serial communication between our robot hands and the Planner PC, is implemented with the Robot Operating System (ROS). The Planner PC runs two nodes, the client node (Main.py) and the server node (stdServo.py or robotHand.py). The client node (Main.py), receives from the user the angle of the servo motor. The server node, sends the desired angle to the robot hand.

# ROS Installation

Install this ROS package placing the folder in the /src directory of your ROS workspace. Build the package using the following command:

$catkin_make

You have successfully installed the package. Make sure that all files are executable, using the following commands:

$chmod +x Main.py
$chmod +x stdServo.py
$chmod +x robotHand.py


# Arduino Installation

Upload with the [arduino IDE](http://arduino.cc/en/main/software) the program to the arduino Micro board.

For the Dynamixel AX12 servo we use this [library](http://savageelectronics.blogspot.gr/2011/01/arduino-y-dynamixel-ax-12.html)

# How to run ROS nodes

To run this ROS package, you have to run one of the following launch files:

$roslaunch openbionics stdServo.launch
$roslaunch openbionics HandAX12.launch

In the launch file you can set up the USB port, to establish serial communication with the robot hand. For the robot hand with the AX12 servo, the server node returns the state of the servo motor: 1) the goal position, 2) the current position and 3) the load. In order to do so, the robotHand.py node publishes the state of the motor, in the Hand ROS topic.

To control the robot hand you can use the following command:

$rosrun openbionics Main.py arg1 arg2 arg3

Replace the arg1 with STD or AX12 depending on your servo. Replace the arg2 with the command that you want send to robot hand (e.g., PS is used for position control of the servo). Replace the arg3 with the desired joint angle value (the ranges are: 0 – 218 for stdServo, 100 – 1023 for AX12).
