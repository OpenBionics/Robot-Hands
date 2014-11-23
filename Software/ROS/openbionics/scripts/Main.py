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

if __name__ == "__main__":
	if len(sys.argv) == 3:
		aperture = float(sys.argv[2])
		cmd = str(sys.argv[1])
		if (cmd == 'PS'):
			cmd = 'PS'
			stdServo_client(cmd, aperture)