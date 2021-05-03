#!/usr/bin/env python3

#rospy is a pure Python client library for ROS
import rospy

#"Float32" msg type is imported from standard messages
from std_msgs.msg import Float32

#random module is imported to generate random values for dirt amount
import random


#publisher function for dirt
def pub_dirt_val():

	#initializing a dirt publisher node
	rospy.init_node('dirt_pub_node')

	#creating a publisher class object to publish dirt values
	# pusblish topic 'dirt' as datatype of Float32 whereas 'queue_size' defines outgoing message queue used for asynchronous publishing of the dirt publisher node
	dirt_pub = rospy.Publisher('dirt', Float32, queue_size=10)

	#setting rate of frequency as 5 times per second
	rate = rospy.Rate(5)

	#While ros is still running
	while not rospy.is_shutdown():
		#generate a random value between 20 and 25 for dirt amount
		val = random.random()*5 + 20
		#publish the value obtained of the dirt
		dirt_pub.publish(val)
		#sleeping 0.2 seconds since Rate is 5 Hz
		rate.sleep()


if __name__ == "__main__":
	try:
		#call the dirt publisher function
		pub_dirt_val()

	#catch only ROS related exceptions and print them out
	except rospy.ROSInterruptException as e:
		print(e)
