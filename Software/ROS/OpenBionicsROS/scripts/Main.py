#!/usr/bin/env python

import roslib; roslib.load_manifest('openbionics')

import sys

import rospy
from openbionics.srv import *

def robotHand_client(req):
	rospy.wait_for_service('robotHand_srv')
	try:
		robotHand_srv = rospy.ServiceProxy('robotHand_srv', hand)
 		resp = robotHand_srv(req)
 		return resp
 	except rospy.ServiceException, e:
 		print "Service call failed: %s"%e

if __name__ == "__main__":
	if len(sys.argv) == 2:
		aperture = float(sys.argv[1])
		if (aperture < 0 or aperture > 1):
			print "[ERROR]: Bad input [0,1]"
			sys.exit(1)
	else:
		print "[ERROR]: Bad input [0,1]"
		sys.exit(1)
   	
	print "%s"%robotHand_client(aperture)
