#!/usr/bin/env python3

#rospy is a pure Python client library for ROS
import rospy

#"Float32" msg type is imported from standard messages
from std_msgs.msg import Float32

# intializing the speed to start off with 0
SPEED = 0


#callback function for the subscriber of '/change_speed' topic
def update_speed(val):
	#defining the global variable
	global SPEED
	#updating the speed to reflect the current speed value with the datatype of float32 as set by round function
	SPEED = round(val.data, 1)


#publisher function
def pub_speed_val():
	#defining the global variable
	global SPEED

	#creating a publisher class object to publish speed values
	#publish the topic 'speed' as datatype of Float32 where the 'queue_size' is defined outgoing message queue used for asynchronous publishing
	speed_pub = rospy.Publisher('speed', Float32, queue_size=10)

	#setting the frequency of 10 times per second
	rate = rospy.Rate(10)

	#while the ros is still running
	while not rospy.is_shutdown():
		#publishing the speed as datatype of Float32
		speed_pub.publish(SPEED)
		rate.sleep()


if __name__ == "__main__":
	try:
		#initializing this speed_publisher node
		rospy.init_node('speed_pub_node')

		#creating a subscriber for the topic "change_speed"
		#received the topic change_speed as data type of Float32 where  the callback func is defined as update_speed
		rospy.Subscriber("change_speed", Float32, update_speed)

		#call publisher for speed
		pub_speed_val()
		
		#to continiously run this node
		rospy.spin()

	#catch only ROS related exceptions and print them out
	except rospy.ROSInterruptException as e:
		print(e)
