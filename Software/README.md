# Introduction

The serial communication between our robot hands and the Planner PC, is implemented with Robot Operating System (ROS). 
The planner-PC runs two nodes, the client node (Main.py) and the server node (stdServo.py).
The client node (Main.py), receives from the user the angle of RC servo. 
The server node (stdServo.py), sends the desired angle to the robot hand. 

# ROS Installation

Install this ROS package, placing the folder, in your ROS workspace in /src directory. 
Then you must write in terminal 
	
	$catkin_make
	
You have successfully installed the package. Also the script file’s mode must be executable.

	$chmod +x Main.py
	$chmod +x stdServo.py

# Arduino Installation

Upload with [arduino IDE](http://arduino.cc/en/main/software) the program to arduino Micro board. 

# How to run ROS nodes

To run this ROS package you must run the launch file.

	$roslaunch openbionics stdServo.launch 

In the launch file (stdServo.launch) you can set up the usb port for serial commumnication with robot hand.	 
	
To send the servo angle to the robot hand you have to run the command

	$rosrun openbionics Main.py arg
	 
Replace the arg with a desired number between 0 and 218 degrees.
