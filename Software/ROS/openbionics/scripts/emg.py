#!/usr/bin/env python
from openbionics.msg import Command, MotorState, Acknowledge, EMGValues
import rospy
from serial import *
import time

def emgRead():

	cnt = 0
	LearningNum = 1000.0
	emgThresFree = 0
	emgThresTight = 0
	emgThres = 0
	ServoAngle = 0

	ServoCmd = rospy.Publisher('RobotHandCmd', Command,
							queue_size = 100)
	emgPublisher = rospy.Publisher('EMGRead', EMGValues,
							queue_size = 100)
	rospy.init_node('emg', anonymous=True)
	rate = rospy.Rate(200) # 200hz
	while not rospy.is_shutdown():
		emgValue = emgSerial("gs")
		if (int(emgValue[0]) == 0):
			# Learnign phase
			if (cnt <= LearningNum/2):
				print "Leave your muscle free"
				time.sleep(0.005)
				emgThresFree = emgThresFree + float(emgValue[1])
			elif (cnt > LearningNum/2 and cnt <= LearningNum):
				print "Tighten your muscle"
				time.sleep(0.005)
				emgThresTight = emgThresTight + float(emgValue[1])
			# Working phase
			else:
				emgThres = (emgThresTight/(LearningNum/2)+
							emgThresFree/(LearningNum/2))/2
				print emgThres
				# Send to Robot Hand the Command
				if (float(emgValue[1]) >= emgThres):
					if (ServoAngle == 0):
						ServoAngle = 100
					elif (ServoAngle == 100):
						ServoAngle = 0
				#elif (float(emgValue[1]) < emgThres):
				#	ServoAngle = 0
				cmd = 'ps %s' % ServoAngle  
				ServoCmd.publish(cmd)
				rate.sleep()
			emgPublisher.publish(float(emgValue[1]))
			cnt = cnt+1

def emgSerial(cmd):
	emgMCU.open()
	# Write data package
	emgMCU.write(str(cmd))
	emgMCU.write('\n')
	# Read data package
	InData = emgMCU.readline()[:-1].split(';')
	emgMCU.close()
	return InData

if __name__ == '__main__':
	try:
		emgMCU = Serial(port = sys.argv[1], baudrate = 115200,
					bytesize = EIGHTBITS, parity = PARITY_NONE,
					stopbits = STOPBITS_ONE, timeout=None)
	except SerialException as e:
		print e
		sys.exit(1)
	emgMCU.close()
	try:
		emgRead()
	except rospy.ROSInterruptException:
		pass
