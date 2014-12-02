#!/usr/bin/env python
 
from openbionics.srv import *
import rospy
from std_msgs.msg import String


from serial import *
import time 

def handle_robotHand_srv(req):
	# Write data package
	SerialWrite(req.cmd, req.Angle)
	#Read data package
	time.sleep(0.1)	
	robotHand.open()
	inByte = robotHand.readline()
	data = str(inByte).split(';')
	pos = float(data[0])
	load = float(data[1])
	robotHand.close()
	# Publish Data to Hand Topic
	state = [req.Angle, pos, load]
	publisher(state)
	# Return Data to Client
	return HandAX12Response(pos, load)
	
# Publish Data to Hand Topic
def publisher(State):
	pub = rospy.Publisher("Hand", String, queue_size=10)
	r = rospy.Rate(400)
	pub.publish(str(State))
	r.sleep()

# Send angle to robot hand 
def SerialWrite(tx1, tx2):
	#Send to serial	
	robotHand.open()
	robotHand.write(str(tx1))
	robotHand.write(str(tx2))
	robotHand.write("\n")
	robotHand.close()

# Initialize ROS Node
def robotHand_service():
	rospy.init_node('robotHand_service')
    	s = rospy.Service('robotHand_srv', HandAX12, handle_robotHand_srv)
    	print "[INFO] robotHand is ready"
    	rospy.spin()

# Main Function 
if __name__ == "__main__":
	global robotHand
	usb = sys.argv[1]
	try:
		robotHand = Serial(usb, 115200, timeout=2)
	except serial.SerialException:
		sys.exit(1)
	# Close the Serial Port
	robotHand.close()
	# Call robot Hand service
   	robotHand_service()