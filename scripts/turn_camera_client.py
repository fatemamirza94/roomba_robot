#!/usr/bin/env python3

#rospy is a pure Python client library for ROS
import rospy

#import TurnCamera and TurnCameraResponse message types from the srv folder
from roomba_robot.srv import TurnCamera, TurnCameraResponse

#import opencv modules and packages for python for displaying the images
import cv2

# import CvBridge module which converts between ROS Image messages and OpenCV images.
from cv_bridge import CvBridge

#import built-in modules and packages
import os
import numpy as np


#define the function for angle to turn image at
def config_request(angle):

#wait for the serice to connect
	rospy.wait_for_service('/turn_camera')

	try:
		#Service definitions container for the request and response type forn turning the camera at a particular angle
		service_proxy = rospy.ServiceProxy('/turn_camera', TurnCamera)
		resp_msg = service_proxy(angle)


		#send response of the image as message
		image_msg = resp_msg.image
		
		#convert a ROS image message into an cv
		#Since the default value of "passthrough" is given, the destination image encoding will be the same as the image message encoding
		image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding='passthrough')


		#After successfully converted images to OpenCV format, see a HighGui window with the desired image will be displayed. 
		cv2.imshow('Turn Camera Image', image)
		
		#handling keyboard interrupt
		cv2.waitKey(0)
		cv2.destroyAllWindows()

#catch only ROS Service related exceptions and print them out
	except rospy.ServiceException as e:
		print("Service Reuqest Failed")
		print(e)


if __name__ == "__main__":
	try:
	#initialize this camera client
		rospy.init_node("turn_camera_client_node")
		
	#prompt user for angle input
		user_input = input("\nEnter the angle: ")

	#continue trying to process angle requirest until user presses q for quitting
		while user_input != 'q':
			try:
			#unless error occurs continue asking user for angles which will be in the float32 datatype
				config_request(float(user_input))
				user_input = input("\nEnter the angle: ")

#if any error while trying to process the request, an exception will be thrown 
			except Exception as e:
				print("Error trying to process request")
				print(e)

#catch only ROS related exceptions and pass
	except rospy.ROSInterruptException:
		pass
