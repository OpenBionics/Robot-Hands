#!/usr/bin/env python
 
from openBionics.srv import *
import rospy

from serial import *
import time 

def openserial():
	global usb
	port = Serial(usb, 115200, timeout=2)
	return port

def handle_robotHand_srv(req):
	robotHand = openserial()
	robotHand.open()
	time.sleep(0.1)
	#Write data package
	robotHand.write(str(req.Aperture))
	robotHand.write("\n")		
	robotHand.close()
	print "[INFO] Send to robotHand"
	print "%s"%(req)
	#return handResponse(load)

def robotHand_srv_server():
	rospy.init_node('robotHand_srv_server')
    	s = rospy.Service('robotHand_srv', hand, handle_robotHand_srv)
    	print "[INFO] robotHand is ready"
    	rospy.spin()

if __name__ == "__main__":
	if len(sys.argv) == 2:
		usb = sys.argv[1]
	else:
		print "[ERROR] Bad Input (e.g. /dev/ttyUSB0)"
		sys.exit(1)
   	robotHand_srv_server()
