#!/usr/bin/env python

import roslib; roslib.load_manifest('openbionics')

import sys

import rospy
from openbionics.srv import *

# Client for robot hand with stdservo
def stdServo_client(req1, req2):
	rospy.wait_for_service('stdServo_srv')
	try:
		stdServo_srv = rospy.ServiceProxy('stdServo_srv', stdServo)
 		resp = stdServo_srv(req1, req2)
 	except rospy.ServiceException, e:
 		print "Service call failed: %s"%e

# Client for robot hand with AX12 servo
def robotHand_client(req1, req2):
	rospy.wait_for_service('robotHand_srv')
	try:
		robotHand_srv = rospy.ServiceProxy('robotHand_srv', HandAX12)
 		resp = robotHand_srv(req1, req2)
 		return resp
 	except rospy.ServiceException, e:
 		print "Service call failed: %s"%e

if __name__ == "__main__":
	if len(sys.argv) == 4:
		# Servo model: AX12 or STD
		model = str(sys.argv[1])
		# Command PS or FC (todo)
		cmd = str(sys.argv[2])
		# Angle of servo or desire force
		set_point = float(sys.argv[3])
		if (model == 'STD'):
			if (cmd == 'PS'):
				stdServo_client(cmd, set_point)
			else:
				print "[ERROR]: Bad input"
				sys.exit(1)
		elif (model == 'AX12'):
			if (cmd == 'PS'):
				print "%s"%robotHand_client(cmd, set_point)
			elif (cmd == 'FC'):
				print "%s"%robotHand_client(cmd, set_point)
			else:
				print "[ERROR]: Bad input"
				sys.exit(1)
		else:
			print "[ERROR]: Wrong Command"
			sys.exit(2)
	else:
		print "[ERROR]: Wrong number of arguments"
		sys.exit(0)
