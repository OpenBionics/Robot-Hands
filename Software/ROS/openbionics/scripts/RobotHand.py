#!/usr/bin/env python
from openbionics.msg import Command, MotorState, Acknowledge
import rospy
from serial import *

class RobotHand():
    def __init__(self):
        try:
            self.MCU = Serial(
                        port = sys.argv[1], baudrate = 115200,
                        bytesize = EIGHTBITS, parity = PARITY_NONE,
                        stopbits = STOPBITS_ONE, timeout=None)
        except serial.SerialException as e:
            print e
            sys.exit(1)
        self.MCU.close()

        self.rate = rospy.Rate(100)
        self.CommandSub = rospy.Subscriber(
                        'RobotHandCmd', Command, self.CommandCB)
        self.MotorStatePub = rospy.Publisher(
                           'MotorState', MotorState, 
                           queue_size = 10)
        self.RobotHandAckPub = rospy.Publisher(
                             'RobotHandAck', Acknowledge,
                             queue_size = 10)

    def CommandCB(self, OutData):
        self.MCUCommunication(OutData.cmd)
        self.ack = Acknowledge()
        self.ack.ack = int(self.InData[0])
        self.RobotHandAckPub.publish(self.ack)
        if len(self.InData) > 1 :
            self.state = MotorState()
            self.state.position = float(self.InData[1])
            self.state.load = float(self.InData[2])
            self.MotorStatePub.publish(self.state)   
        self.rate.sleep()

    def MCUCommunication(self, cmd):        
        # Write data package
        self.MCU.open()
        self.MCU.write(str(cmd))
        self.MCU.write('\n')
        # Read data package
        self.InData = self.MCU.readline()
        self.InData = self.InData[:-1].split(';')
        self.MCU.close()

if __name__ == '__main__': 
    rospy.init_node('RobotHand')

    Hand = RobotHand()

    while not rospy.is_shutdown():
        rospy.spin()
