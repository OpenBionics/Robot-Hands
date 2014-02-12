# Introduction

The serial communication between our robot hands and the Planner PC, is implemented with Robot Operating System (ROS). 

# OpenBionics ROS Package Directions

The planner-PC runs two nodes, the client node (Main.py) and the service node (robotHand.py).
The client node (Main.py), receives from the user the aperture value 
(0 when the hand is fully open and 1 when the hand is fully close).
The service node (robotHand.py), sends the desired aperture to the robot hand. 

# ROS Installation

Install this ROS package, placing the folder, in your ROS workspace in /src directory. 
Then you must write in terminal 
	
	$catkin_make
	
You have successfully installed the package.

# Arduino Installation

Upload with [arduino IDE](http://arduino.cc/en/main/software) the program to arduino Micro board. 

# How to run ROS nodes

To run this ROS package at first you must run 

	roscore
	
Then in another terminal tab you have to execute as super-user the command 
	
	#rosrun openbionics/robotHand.py serial_port 

Replace <serial_port> with the port where your arduino is connected e.g /dev/ttyACM0. 

To send the aperture value to the robot hand you have to run the command
	$rosrun openbionics/Main.py <aperture> 
Replace the <aperture> with a desired number between 0 and 1.
