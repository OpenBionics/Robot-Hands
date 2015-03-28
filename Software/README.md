# Introduction

<p style='text-align: justify;'>
In order to control the robot hand, we use the Robot Operating System (ROS) and arduino micro board.
The Planner PC runs one ROS node (RobotHand.py), that is responsible for sending commands, for parsing the data packages which are send by 
arduino micro and for publishing the received information in the ROS topics.
The arduno micro is responsible to parse the planner pc commands, to drive the different servo motors and is respond to the planner pc with
the information about robot hand.
</p>

     # ROS Installation

Install this ROS package placing the folder in the /src directory of your ROS workspace. Build the package using the following command:

    $catkin_make

You have successfully installed the package. Make sure that all files are executable, using the following command:

     $chmod +x RobotHand.py

# Arduino Installation

Donwload the [arduino IDE](http://arduino.cc/en/main/software).
Place the [library for dynamixel servo](https://github.com/OpenBionics/Robot-Hands/tree/master/Software/Arduino/Libraries/DynamixelSerial) in 
your library folder. 
Place the RobotHand folder in your arduino workspace.
Open the RobotHand.ino with arduino IDE.
Select with MODE definition the servo motor that you use:

    #define MODE 1 :is for stadard RC servo
    #define MODE 2 :is for dynamixel AX12 servo

Upload the program to the arduino Micro board.

# How to run ROS node

To run this ROS package, you have to run the launch file:

    $roslaunch openbionics RobotHand.launch

In the launch file you can set up the USB port, to establish serial communication with the robot hand. It is necessary to give write permissions
on that port. The RobotHand node listen to RobotHandCmd topic and send those commands to arduino board via serial. The robot hand responds with 
sending back to the host pc the acknowledgement and the state of servo motor (depending to the sending command). The RobotHand node publish the 
acknowledgment in the RobotHandAck topic and the state of motor in MotorState topic.
If the acknowledgment is 0 means that robot hand receives acceptable command, but if it is -1 means that you have send wrong command. The motor 
state package has the following format:
    
    acknowledgment;angle (deg);load (0-1023 for CCW, 1024-2047 for CW)\n

# How to control the robot hand

To control the robot hand you can use the following command:

    $rostopic pub -1 /RobotHandCmd openbionics/Command "cmd: 'arg'" 

Replace the arg: 

    ps "angle in deg", the angle depends on the servo motor
    vl "velocity in deg/s", the velocity depends on the servo motor
    gs, to get the state of servo motor

In order to view the acknowledgment and the state of motor you can use the following commands:

	$rostopic echo /MotorState
	$rostopic echo /RobotHandAck
