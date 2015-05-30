#!/usr/bin/env python
from openbionics.msg import Command, MotorState, Acknowledge
import rospy
from serial import *

def SliderBox():
    SliderBoxCmd = rospy.Publisher(
                 'RobotHandCmd', Command, 
                 queue_size = 10)
    rospy.init_node('SliderBox', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        # Send Command to SliderBox and Read the Response
        cmd = SliderBoxSerial("gs")
        cmd  = cmd[:-1].split(';')
        if (int(cmd[0]) == 0):
            # Send to Robot Hand the Command
            cmd = 'ps %s' % cmd[1] 
            SliderBoxCmd.publish(cmd)
            rate.sleep()
        
def SliderBoxSerial(cmd):        
    # Write data package
    SliderBoxMCU.open()
    SliderBoxMCU.write(str(cmd))
    SliderBoxMCU.write('\n')
    # Read data package
    InData = SliderBoxMCU.readline()
    SliderBoxMCU.close()    
    return InData

if __name__ == '__main__':
    try:
        SliderBoxMCU = Serial(
                    port = sys.argv[1], baudrate = 115200,
                    bytesize = EIGHTBITS, parity = PARITY_NONE,
                    stopbits = STOPBITS_ONE, timeout=None)
    except SerialException as e:
        print e
        sys.exit(1)
    SliderBoxMCU.close()
    try:
        SliderBox()
    except rospy.ROSInterruptException:
        pass