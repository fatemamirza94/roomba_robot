#!/usr/bin/env python3

#rospy is a pure Python client library for ROS
import rospy

#import RobotPosition to access pos_x and pos_y for Roomba 
from roomba_robot.msg import RobotPosition

#Define and initialize global variables 
POSITION = RobotPosition()
#Access and assign initial values from rosparam server
POSITION.pos_x = rospy.get_param('initial_post_x')
POSITION.pos_y = rospy.get_param('initial_post_y')


#callback function for the subscriber of '/change_position' topic
def update_position(data):
	#defining the global variable
	global POSITION
	#updating the global variable value with current position value to always keep the last position
	POSITION = data


#publisher function
def pub_position_val():
	#defining the global variable
	global POSITION

	#creating a publisher class object to publish position values
	#publish topic 'position' as msgtype of RobotPosition where'queue_size' is defined as outgoing message queue used for asynchronous publishing
	position_pub = rospy.Publisher('position', RobotPosition, queue_size=10)

	#setting rate of frequency at 10 times per second
	rate = rospy.Rate(10)

	#while ros is still running
	while not rospy.is_shutdown():
		#publishing the position whose msg type is of the type RobotPosition
		position_pub.publish(POSITION)
		rate.sleep()


if __name__ == "__main__":
	try:
		#initializing this position publisher node
		rospy.init_node('position_pub_node')

		#creating a subscriber for the topic "change_position"
		#received change_position as data type of RobotPosition from callback func of update_position
		rospy.Subscriber("change_position", RobotPosition, update_position)

		#call publisher for position
		pub_position_val()
		
		#to continiously keep this node running
		rospy.spin()

	#catch only ROS related exceptions and print them out
	except rospy.ROSInterruptException as e:
		print(e)
