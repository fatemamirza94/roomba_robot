#!/usr/bin/env python3

#rospy is a pure Python client library for ROS
import rospy

#actionlib module is needed to create the Action client
import actionlib

#"Float32" msg type is imported from standard messages
from std_msgs.msg import Float32


#msg types are needed to enable for Action client and
#subsequent Subscriber to position topic
#They are imported from the messages that have been created before by the author
from roomba_robot.msg import RobotPosition
from roomba_robot.msg import PositionAndDestination

#msg types required to communicate with Action Server are imported from msgs folder
from roomba_robot.msg import WalkToPointAction
from roomba_robot.msg import WalkToPointResult
from roomba_robot.msg import WalkToPointFeedback
from roomba_robot.msg import WalkToPointGoal

#importing built-in packages and modules
import os
import math


#creating menu for selection of task for roomba
menu = """
############## Select options by pressing 1-4 on your keyboard ##############

>>>>> 1. Current Dirt Amount
>>>>> 2. Current Roomba Position
>>>>> 3. Traverse Roomba to a Specified Destination
>>>>> 4. Return Roomba to the charging station
"""


#Initializing Global variables with default and rosparam server values
CUR_DIRT = 0
CUR_ROB_POS = [rospy.get_param('initial_post_x'), rospy.get_param('initial_post_y')]
ROB_BASE_POINT = [rospy.get_param('initial_post_x'), rospy.get_param('initial_post_y')]
CUR_ROB_SPEED = 0
ROB_MAX_SPEED = rospy.get_param("robot_max_speed")

#getting range values for the roomba
MAX_CORDINATE = rospy.get_param("max_cordinate")
MIN_CORDINATE = rospy.get_param("min_cordinate")
# ------------------------------------------------------


#funtion to set up the environment and initialize values
def setup():
	#setting up the menu as global
	global menu
	#clear previous things on the screen
	clear()



#main function of Control Node
def main():


	#main loop of the control_node
	while True:
		#printing menu variable to the screen to show options to the user
		print(menu)

		#Validity check for input
		check = True
		while check:
			try:
				#asking the user for input
				choice = int(input("Select: "))
				#cheking if the input in this list, otherwise AssertionError will be raised
				assert choice in [1, 2, 3, 4]
				check = False
			except:
				print("Please choose a valid number")

		#if choice is 1, let the user know about the amount of dirt in the position
		if choice == 1:
			print("Current DIRT: {} Amount".format(CUR_DIRT))


		#if choice is 2, let the user know about the current position of robot
		elif choice == 2:
			print("Current Position: X = {}  Y = {}".format(CUR_ROB_POS[0], CUR_ROB_POS[1]))


		#if choice is 3, action client is used to go to the desired position
		elif choice == 3:
			#from positionanddestination get the goal
			goal = PositionAndDestination()
			print("Cordinations of Destination")


			#to ensure that the value is within the range
			x = check_input(" X is equal to  ")
			y = check_input(" Y is qual to  ")

			#setting goal msg data
			goal.dest_x = x
			goal.dest_y = y
			goal.cur_x = CUR_ROB_POS[0]
			goal.cur_y = CUR_ROB_POS[1]

			#calling the walk_to_point to create an Action Client
			walk_to_point(goal)

		#if choice is 4, use action client to go to the charging station of robot
		else:
			#creating the goal message
			goal = PositionAndDestination()
			#setting goal msg data
			goal.dest_x = 0
			goal.dest_y = 0
			goal.cur_x = CUR_ROB_POS[0]
			goal.cur_y = CUR_ROB_POS[1]

			#calling the walk_to_point to create an Action Client
			walk_to_point(goal)


#Walk To Point Action client function
def walk_to_point(message):
	#defining global variables
	global ROB_MAX_SPEED
	global CUR_ROB_POS

	#equation to determing is the robot is in the final destination
	if message.dest_x == message.cur_x and message.dest_y == message.cur_y:
		print("Roomba is in the final destination")
	#otherwise create an action client to walk to the final destination
	else:
		#creating action client; action name is 'walk_to_point'
		#msg type is WalkToPointAction
		walk_client = actionlib.SimpleActionClient('walk_to_point', WalkToPointAction)
		#waiting for Action server to respond
		walk_client.wait_for_server()
		#create a goal message for walking
		destination = WalkToPointGoal(message)

		#Let the user know about distance and time to reach the final destination
		distance = round(math.dist([message.cur_x, message.cur_y], [message.dest_x, message.dest_y]), 1)
		time_to_reach = int(distance/ROB_MAX_SPEED)
		print("Distance: {} meters and the Time Tken: {} s".format(distance, time_to_reach))

		#sending the goal to the Action server, feedback callback function is process_feedback
		#Feedback callback function creates a thread for itself
		walk_client.send_goal(destination, feedback_cb = process_feedback)
		#wait for the Action server to finish the task to get the result
		walk_client.wait_for_result()

		rospy.sleep(0.7)

		#clear the terminal screen
		clear()

		#printing the current position of roomba
		print("Current Cordinations position: X = {}  Y = {}".format(CUR_ROB_POS[0], CUR_ROB_POS[1]))


#action client feedback callback funtion
def process_feedback(feedback):
	#accessing gloabl variable
	global CUR_ROB_SPEED
	#Let user know abot the distance to reach final destination
	print("Current Distance: {} m moving at Speed: {} m/s".format(round(feedback.distance, 1), CUR_ROB_SPEED))





#callback function for the subscriber of '/dirt' topic
def dirt_sub(val):
	#defining the global variable
	global CUR_DIRT
	#updating the global variable value to a the float number using built in round function
	CUR_DIRT = round(val.data, 1)





#callback function for the subscriber of '/position' topic
def rob_pos_sub(val):
	#defining the global variable
	global CUR_ROB_POS
	#updating the global variable value to a the float number using built in round function
	CUR_ROB_POS[0] = round(val.pos_x)
	CUR_ROB_POS[1] = round(val.pos_y)


#callback function for the subscriber of '/speed' topic
def rob_speed_sub(val):
	#defining the global variable
	global CUR_ROB_SPEED
	#updating the global variable value to a the float number using built in round function
	CUR_ROB_SPEED = round(val.data, 1)


#clearing the user terminal  function
def clear():
	# for mac and linux (os.name is 'posix')
	if os.name == 'posix':
		_ = os.system('clear')
		print()
	else:
	# for windows platfrom
  		_ = os.system('cls')
  		print()


#auto-validate user input values
def check_input(asked):
	#defining the global variable
	#the range of roomba operation
	global MAX_CORDINATE
	global MIN_CORDINATE
	#Flag variable to keep track of whether the user input is valid
	flag = True

	while flag:
		try:
			#User input for valid position
			number = float(input(asked))
			#checking the input value is within the acceptable rance
			assert MIN_CORDINATE <= number and number <= MAX_CORDINATE, "Select from {} to {}"
			#if there is no AssertionError,the loop is stopped
			flag = False

		#if AssertionError occurs, raise an excetion and print an error has occured
		except AssertionError as msg:
			print("Error")

		#if other exception arises, ask user to reenter the input
		except:
			print("Enter a valid number")

	#return the valid user input
	return number
if __name__ == "__main__":
	try:
		#initializing a control node
		rospy.init_node("control_node")

		#creating a subscriber for the topic "dirt"
		#received dirt as data type of Float32 from the callback func named dirt_sub
		rospy.Subscriber('dirt', Float32, dirt_sub)
		#creating a subscriber for the topic "position"
		#received position as data type of RobotPosition from the callback func named rob_pos_sub
		rospy.Subscriber('position', RobotPosition, rob_pos_sub)
		#creating a subscriber for the topic "speed"
		#received spped as data type of Float32 from the callback func named rob_speed_sub
		rospy.Subscriber('speed', Float32, rob_speed_sub)


		#launching setup function to set up the environment
		setup()
		#main function of Control Node - this is from where all the execution begins
		main()

		#to continiously run this control_node
		rospy.spin()

	#catch only ROS related exceptions and print them out
	except rospy.ROSInterruptException as e:
		print(e)
