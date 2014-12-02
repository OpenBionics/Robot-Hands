#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
	# Write the State of Hand to file
	f = open("/home/azisi/Desktop/measures.txt","a")
	f.write((data.data))
	f.write("\n")
	print data.data
	f.close()
    
def StateSubscriber():
    rospy.init_node('StateSubscriber', anonymous=True)
    rospy.Subscriber("Hand", String, callback)
    rospy.spin()

if __name__ == '__main__':
	StateSubscriber()