#!/usr/bin/env python

import roslib; roslib.load_manifest('openbionics')

import sys

import rospy
from openbionics.srv import *

def stdServo_client(req1, req2):
	rospy.wait_for_service('stdServo_srv')
	try:
		stdServo_srv = rospy.ServiceProxy('stdServo_srv', stdServo)
 		resp = stdServo_srv(req1, req2)
 	except rospy.ServiceException, e:
 		print "Service call failed: %s"%e

def robotFinger_client(req):
	rospy.wait_for_service('robotFinger_srv')
	try:
		robotFinger_srv = rospy.ServiceProxy('robotFinger_srv', Finger)
 		resp = robotFinger_srv(req)
 		return resp
 	except rospy.ServiceException, e:
 		print "Service call failed: %s"%e

if __name__ == "__main__":
	if len(sys.argv) == 3:
		aperture = float(sys.argv[2])
		cmd = str(sys.argv[1])
		if (cmd == 'PS'):
			cmd = 'PS'
			stdServo_client(cmd, aperture)
		if (cmd == 'H'):
			if (aperture < 0 or aperture > 10):
				print "[ERROR]: Bad input [0,10]"
				sys.exit(1)
			else:
				print "%s"%robotHand_client(aperture)
		elif(cmd == 'F'):
			if (aperture < 0 or aperture > 100):
				print "[ERROR]: Bad input [0,100]"
				sys.exit(1)
			else:
				print "%s"%robotFinger_client(aperture)
