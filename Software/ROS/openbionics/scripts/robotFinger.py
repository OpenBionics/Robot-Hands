#!/usr/bin/env python
 
from openbionics.srv import *
import rospy

from serial import *
import time 

def openserial():
	global usb
	port = Serial(usb, 115200, timeout=2)
	return port

def handle_robotFinger_srv(req):
	f = open("/home/azisi/Desktop/measures.txt","a")
	robotFinger = openserial()
	robotFinger.open()
	time.sleep(0.1)
	#Write data package
	robotFinger.write(str(req.Aperture))
	robotFinger.write("\n")		
	robotFinger.close()
	print "[INFO] Send to robotFinger"
	print "%s"%(req)
	#Read data package
	time.sleep(0.1)	
	robotFinger.open()
	f.write("%.3f \t" %(req.Aperture))
	#print "[INFO] Receive from robotHand"
	inByte = robotFinger.readline()
	data = str(inByte).split(';')
	pulse = float(data[0])
	f.write("%.3f \n" %(pulse))	
	f.close()
	robotFinger.close()
	return fingerResponse(pulse)

def robotFinger_srv_server():
    rospy.init_node('robotFinger_srv_server')
    s = rospy.Service('robotFinger_srv', Finger , handle_robotFinger_srv)
    print "[INFO] finger is ready"
    rospy.spin()

if __name__ == "__main__":
	if len(sys.argv) == 2:
		usb = sys.argv[1]
	else:
		print "[ERROR] Bad Input (e.g. /dev/ttyUSB0)"
		sys.exit(1)
   	robotFinger_srv_server()
