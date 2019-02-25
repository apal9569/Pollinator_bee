#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from plutodrone.msg import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32,Float64 
import time

class image_detect:
	
	def __init__(self):

		rospy.init_node('ros_bridge')
		
		#creating a ros bridge
		self.red = rospy.Publisher('/red',Int32 , queue_size=10)
		self.ros_bridge = cv_bridge.CvBridge()
		#subscribing to the /usb_cam/image_rect_color for getting the frames
		self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, self.image_call)
				

	#taking a frame at a time, detecting the LEDs of red blue and green and drawing contour around them
	def image_call(self,msg):

		#converting image for CV2 operation
		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		#converting RGB image to HSV 
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		#setting the lower HSV values for green LEDs
                lower_g = np.array([22, 103, 91])
		#setting the upper HSV values for green LEDs
                upper_g = np.array([65, 161, 162])
		#setting the lower HSV values for Blue LEDs
               	lower_b = np.array([112, 44, 166])
		#setting the upper HSV values for Blue LEDs
               	upper_b = np.array([141, 126, 233])
		#setting the lower HSV values for red LEDs
               	lower_r = np.array([0, 73, 129])
		#setting the upper HSV values for red LEDs
              	upper_r = np.array([19,159 , 192])
		#using the mask functions for getting only the images of LEDs
              	greenMask = cv2.inRange(hsv, lower_g, upper_g)#for green
               	blueMask = cv2.inRange(hsv, lower_b, upper_b)#for blue
               	redMask = cv2.inRange(hsv, lower_r, upper_r)#for red
		#using findContours function for getting all the contrours formed by the LEDs
               	_, greenContours, hierarchy1 = cv2.findContours(greenMask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)#for green
             	_, blueContours, hierarchy2 = cv2.findContours(blueMask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)#for blue
		if (blueContours!=2):
			self.red.publish(1)
              	_, redContours, hierarchy3 = cv2.findContours(redMask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)#for red
		if (len(redContours)>5):
			self.red.publish(1)
               	greenMax = 0;#for maximum area of green
               	blueMax = 0;#for maximum area of blue
               	redMax = 0;#for maximum area of red
               	greenContourIndex = 0
              	blueContourIndex = 0
               	redContourIndex = 0

		#loop for finding the contour where we are getting the for Green LEDs
               	for i in range(len(greenContours)):
                     	if (cv2.contourArea(greenContours[i]) > greenMax):
                       		greenMax = cv2.contourArea((greenContours[greenContourIndex]))
                       		greenContourIndex = i
 		#for forming the rectangle around the green LEDs we used the moments function for find the centroid and cv2.rectangle for forming the rectangle
                if len(greenContours) != 0:
                       	M = cv2.moments(greenContours[greenContourIndex])
                       	if (M['m00']) != 0:
                       		cx = int(M['m10'] / M['m00'])#x coordinate of centroid
                       		cy = int(M['m01'] / M['m00'])#y coordinate of centroid
                            	cv2.rectangle(image, (cx - 7, cy - 5), (cx + 7, cy + 5), (0, 255, 0), 2)

		#loop for finding the contour where we are getting the for Red LEDs
                for i in range(len(redContours)):
            		if (cv2.contourArea(redContours[i]) > redMax):
                		redMax = cv2.contourArea((redContours[redContourIndex]))
               			redContourIndex = i
                #for forming therectangle around the red LEDs we used the moments function for find the centroid and cv2.rectangle for forming the rectangle
              	if len(redContours) != 0:
                        M = cv2.moments(redContours[redContourIndex])
                       	if (M['m00']) != 0:
                       		cx = int(M['m10'] / M['m00'])#x coordinate od centroid
                       		cy = int(M['m01'] / M['m00'])#y coordinate of centroid
                          	cv2.rectangle(image, (cx - 7, cy - 5), (cx + 7, cy + 5), (0, 0, 255), 2)
		
		#loop for finding the contour where we are getting the for blue LEDs
                for i in range(len(blueContours)):
                        if (cv2.contourArea(blueContours[i]) > blueMax):
                              blueMax = cv2.contourArea((blueContours[blueContourIndex]))
                              blueContourIndex = i
                #for forming the rectangle around the blue LEDs we used the moments function for find the centroid and cv2.rectangle for forming the rectangle
                if len(blueContours) != 0:
                        M = cv2.moments(blueContours[blueContourIndex])
                        if (M['m00']) != 0:
                        	cx = int(M['m10'] / M['m00'])#x coordinate of centroid
                    	        cy = int(M['m01'] / M['m00'])#y coordinate of centroid
                    	        cv2.rectangle(image, (cx - 7, cy - 5), (cx + 7, cy + 5), (255, 0, 0), 2)

		#displaying the frame after drawing the rectangular contour in frame
              	cv2.imshow('frame', image)
               	key = cv2.waitKey(5)

if __name__ == "__main__":
	test=image_detect()
	rospy.spin()

