# Introduction

The serial communication between our robot hands and the Planner PC, is implemented with Robot Operating System (ROS). 
The planner-PC runs two nodes, the client node (Main.py) and the server node (stdServo.py or robotHand.py).
The client node (Main.py), receives from the user the angle of servo motor. 
The server node, sends the desired angle to the robot hand. 

# ROS Installation

Install this ROS package, placing the folder, in your ROS workspace in /src directory. 
Then you must write in terminal 
	
	$catkin_make
	
You have successfully installed the package. Also the script file’s mode must be executable.

	$chmod +x Main.py
	$chmod +x stdServo.py
	$chmod +x robotHand.py

# Arduino Installation

Upload with [arduino IDE](http://arduino.cc/en/main/software) the program to arduino Micro board. 

# How to run ROS nodes

To run this ROS package you must run the launch file.

	$roslaunch openbionics stdServo.launch 
    or
    $roslaunch openbionics HandAX12.launch 

In the launch file you can set up the usb port for serial commumnication with robot hand. Also, for robot hand with AX12 servo the server node returns the state of servo motor, goal position, current position and load.
For that, the robotHand.py node publish the state of motor in Hand ROS topic.
	
To move the robot hand you have to run the command

	$rosrun openbionics Main.py arg1 arg2 arg3

Replace the arg1 with the STD or AX12 depending on your servo.
Replace the arg2 with command that you want send to robot hand, for now PS is for position control of servo.
Replace the arg3 with a desired number of angle (for stdServo 0-218, for AX12 100 - 1023).