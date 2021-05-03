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


#class that defines functions that handles image loading, image conversion to messages and subsequently sending those messages to the client
class TurnCameraClass:

	def __init__(self):
	#angles available for calling in the images folder
	#angles here refer to the names of the images stored i nthe folder images
		self.available_angles = [-30, -15, 0, 15, 30]
		
		
	#A Service is created using a rospy.Service instance with a callback (send_image) to invoke when new requests are received for the rosservice turn_camera. Each inbound request is handled in its own thread, so services must be thread-safe. 
		self.ros_service = rospy.Service('turn_camera', TurnCamera, self.send_image)


#function to send image to the client and display
	def send_image(self, req):
	
	#retrieve the requested image with the angle as defined by the user 
		image = self.get_image(req.turn_degrees)
		
	#using the cvbridge module, convert the retrieved image to a message and make it in a format ready to be sent to the client
		image_msg = CvBridge().cv2_to_imgmsg(image)
		return TurnCameraResponse(image_msg)



#function to get the correct image 
	def get_image(self, angle):
	
	#as mentioned above, the angle is the file name. however, if the user inputs a value which is not in the listed available_angles, an equation is developed to match to the closest angle and open a file with that name. For eg: if the user types 1, the image of 0 will appear on the screen.
		closest_angle = min(self.available_angles, key=lambda x: abs(x-angle))

		return self.read_image_file(str(closest_angle) + '.png')

	
#function to read the image from the image folder	
	def read_image_file(self, file_name):
	
#sets the directory path
		dir_name = os.path.dirname(__file__)
		
	#set the file location directory, which is the images folder
		file_location = dir_name + "/Images/" + file_name
		
	#use the python module cv2 function imread to read the image from the file location specified
		image = cv2.imread(file_location)

		return image


if __name__ == '__main__':
	try:
	
	#intialize this node
		rospy.init_node('turn_camera_service_node')
	#call the serivce class to access all the functions defined by this class
		TurnCameraClass()
		print("Service is running")

	#continue running this node
		rospy.spin()

#catch only ROS related exceptions and print the exception
	except rospy.ROSInterruptException as e:
		print(e)
