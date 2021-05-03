#!/usr/bin/env python3

#rospy is a pure Python client library for ROS
import rospy

#import built-in modules
import math
import threading

#import actionlib module to help in creating an Action server
import actionlib


#"Float32" msg type is imported from standard messages
from std_msgs.msg import Float32

#import and create action msg types for action server
from roomba_robot.msg import WalkToPointAction
from roomba_robot.msg import WalkToPointResult
from roomba_robot.msg import WalkToPointFeedback

#import and create msg types for updating positional data
from roomba_robot.msg import PositionAndDestination
from roomba_robot.msg import RobotPosition


#create a class for Action Server
class Walking:

	def __init__(self, pub1, pub2):
		#create an Action server with name: 'walk_to_point'
		#received topic walk_to_point from callback func of reach_destination
		self.action_server = actionlib.SimpleActionServer('walk_to_point', WalkToPointAction, self.reach_destination)
		
		#assign 'speed_change'&'position_change' to the created publisher 
		self.pub_speed_change = pub1
		self.pub_pos_change = pub2
		#create current speed variable
		self.cur_speed = 0
		#return max speed value from rosparam server
		self.robot_max_speed = rospy.get_param("robot_max_speed")
		#setting rate of frequency as 20 times per second
		self.publisher_rate = rospy.Rate(20)

		#creating current and final position objects
		self.robot_cur_point = RobotPosition()
		self.robot_goal_point = RobotPosition()
		

	#callback funtion of Action server
	def reach_destination(self, goal):

		#threads to publish changes in speed and position while performing the action
		#target is the publisher function
		self.thread_speed = threading.Thread(target=self.pub_change_speed)
		self.thread_position = threading.Thread(target=self.pub_change_position)
		
		#flag to continue running the thread
		self.thread_flag = True

		#Extract current position of robot
		self.robot_cur_point.pos_x = goal.destin.cur_x
		self.robot_cur_point.pos_y = goal.destin.cur_y

		#Extract final destination position values
		self.robot_goal_point.pos_x = goal.destin.dest_x
		self.robot_goal_point.pos_y = goal.destin.dest_y

		#calculate the distance to final destination using math.dist
		# time = distance/speed from physics formula
		distance = round(math.dist([self.robot_cur_point.pos_x, self.robot_cur_point.pos_y], [self.robot_goal_point.pos_x, self.robot_goal_point.pos_y]), 1)
		time_to_reach = int(distance/self.robot_max_speed)
		
		#Let the user know about distance and time to reach
		print("Distance: {} meters travelled in the Time: {} s\n".format(distance, time_to_reach))
		
		#Roomba traverses at 45 angle to the final position
		#Defining x_step for each second where roomba walks x_step distance in x coordinate
		if self.robot_cur_point.pos_x < self.robot_goal_point.pos_x:
			step_x = (self.robot_goal_point.pos_x - self.robot_cur_point.pos_x)/time_to_reach
		else:
			step_x = (self.robot_cur_point.pos_x - self.robot_goal_point.pos_x)/time_to_reach
			step_x *= -1

		#Defining y_step for each second where roomba walks y_step distance in y coordinate
		if self.robot_cur_point.pos_y < self.robot_goal_point.pos_y:
			step_y = (self.robot_goal_point.pos_y - self.robot_cur_point.pos_y)/time_to_reach
		else:
			step_y = (self.robot_cur_point.pos_y - self.robot_goal_point.pos_y)/time_to_reach
			step_y *= -1
		
		#assign current speed to max speed 
		self.cur_speed = self.robot_max_speed

		#initiate threads to publish changes in speed and position independently
		self.thread_speed.start()
		self.thread_position.start()
		
		#Let the users know about the remaining distance 
		if time_to_reach < 10:
			interval = 2
		elif time_to_reach < 20:
			interval = 3
		else:
			interval = 4

		#initiate walking
		for i in range(time_to_reach):
			if i%interval == 0:
				#calculating the distance remaining and sending it to Action client as feedback
				distance = math.dist([self.robot_cur_point.pos_x, self.robot_cur_point.pos_y], [self.robot_goal_point.pos_x, self.robot_goal_point.pos_y])
				self.action_server.publish_feedback(WalkToPointFeedback(distance))

			#sleep for the 1 second
			rospy.sleep(1)
			
			#add or subtract the postion cordinates with step_x and step_y values 
			self.robot_cur_point.pos_x += step_x
			self.robot_cur_point.pos_y += step_y

		#after Roomba reaches destination, update current position values with final destination position values
		self.robot_cur_point.pos_x = self.robot_goal_point.pos_x
		self.robot_cur_point.pos_y = self.robot_goal_point.pos_y

		#setting the current speed value to 0
		self.cur_speed = 0

		#letting threads wait to publish the last changed values in speed and position
		rospy.sleep(1)

		#setting thread_flag False to end threads
		self.thread_flag = False

		#ensure threads stopped successfully
		rospy.sleep(0.5)

		#join additional threads to the main thread
		self.thread_speed.join()
		self.thread_position.join()

		#From walktopoint results create Action result object
		result = WalkToPointResult()
		#save value of time spent to reach final destionation
		result.time_spent = time_to_reach
		#set success answer to client with Action Result msg 
		self.action_server.set_succeeded(result)


	#thread function to publish changes in current speed of roomba
	def pub_change_speed(self):

		#checking thread_flag value
		while self.thread_flag:
			#publishing changes
			self.pub_speed_change.publish(self.cur_speed)
			#wait 0.05 second (20 Hz)
			self.publisher_rate.sleep()


	#thread function to publish changes in current position of roomba
	def pub_change_position(self):

		#checking thread_flag value
		while self.thread_flag:
			#publishing changes
			self.pub_pos_change.publish(self.robot_cur_point)
			#wait 0.05 second (20 Hz)
			self.publisher_rate.sleep()


if __name__ == '__main__':
	
	try:
		#initializing this node
		rospy.init_node('walk_to_point_action_server_node')

		#create a publisher class object to publish speed changes
		#publish topic 'change_speed' as datatype of Float32 where 'queue_size' is defined as outgoing message queue used for asynchronous publishing
		pub_speed = rospy.Publisher('change_speed', Float32, queue_size=10)

		#create a publisher class object to publish position changes
		#publish topic 'change_position' as datatype RobotPosition where 'queue_size' -> outgoing message queue used for asynchronous publishing
		pub_position = rospy.Publisher('change_position', RobotPosition, queue_size=10)

		#using publisher objects as arguements create walking class
		server = Walking(pub_speed, pub_position)
		
		#to continiously run this node
		rospy.spin()

	#catch only ROS related exceptions and print them out
	except rospy.ROSInterruptException as e:
		print(e)
