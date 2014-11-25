#!/usr/bin/env python
 
from openbionics.srv import *
import rospy

from serial import *
import time 

def handle_robotHand_srv(req):
	f = open("/home/azisi/Desktop/measures.txt","a")
	robotHand.open()
	#Write data package
	robotHand.write(str(req.Aperture))
	robotHand.write("\n")		
	robotHand.close()
	print "[INFO] Send to robotHand"
	print "%s"%(req)
	#Read data package
	time.sleep(0.1)	
	robotHand.open()
	f.write("%.3f \t" %(req.Aperture))
	#print "[INFO] Receive from robotHand"
	inByte = robotHand.readline()
	data = str(inByte).split(';')
	load = float(data[0])
	#pos = float(data[1])
	pos = -1
	f.write("%.3f \t %.3f \n" %(load,pos))
	
	#print "[INFO] Receive from robotHand"
	#while 1:
	#	inByte = robotHand.readline()
	#	data = str(inByte).split(';')
	#	load = float(data[0])
	#	pos = float(data[1])
	#	if load != -1 and pos != -1:
	#		f.write("%s \t %s \n" %(load,pos))
	#	else:
	#		f.write("\n\n")
	#		break
	
	f.close()
	robotHand.close()
	return handResponse(load,pos)

def robotHand_srv_server():
	rospy.init_node('robotHand_srv_server')
    	s = rospy.Service('robotHand_srv', HandAX12, handle_robotHand_srv)
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

   	robotHand_srv_server()