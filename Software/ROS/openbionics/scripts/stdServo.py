#!/usr/bin/env python
 
from openbionics.srv import *
import rospy

from serial import *
import time 

def handle_stdServo(req):
	serialWrite(req.Angle, req.cmd)
	print "[INFO] Send to robotHand"
	print "%s"%(req)

# Send angle in degrees to servo motor 
def serialWrite(tx1, cmd):
	#Send to serial	
	robotHand.open()
	robotHand.write(cmd)
	robotHand.write(str(tx1))
	robotHand.write("\n")
	robotHand.close()

def stdServo_server():
	rospy.init_node('stdServo_server')
    	s = rospy.Service('stdServo_srv', stdServo, handle_stdServo)
    	print "[INFO] robotHand is ready"
    	rospy.spin()

if __name__ == "__main__":
	global robotHand
	usb = sys.argv[1]
	try:
		robotHand = Serial(usb, 115200, timeout=2)
	except serial.SerialException:
		sys.exit(1)

	robotHand.close()

   	stdServo_server()