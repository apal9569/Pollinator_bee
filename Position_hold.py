#!/usr/bin/env python

from plutodrone.srv import *
from plutodrone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32,Float64
import rospy
import time
import numpy as np


class DroneFly():
	"""docstring for DroneFly"""
	def __init__(self):
		
		#initializing the node to the ROS
		rospy.init_node('pluto_fly', disable_signals = True)
		
		self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
		
		#subscribing to the /whycon/poses for getting the location of drone
		rospy.Subscriber('/whycon/poses', PoseArray, self.get_pose)
		
		rospy.Subscriber('/red', Int32, self.red)
		rospy.Subscriber('/drone_yaw', Float64, self.get_yaw)

		self.cmd = PlutoMsg()

		#previos time
		self.throt_lt=0.0
		self.pitch_lt=0.0
		self.roll_lt=0.0
		self.yaw_lt=0.0
		
		self.red=0

		#time
		self.throt_time=0.0
		self.pitch_time=0.0
		self.roll_time=0.0
		self.yaw_time=0.0

		#error
		self.throt_error=0.0
		self.p_err=0.0
		self.r_err=0.0
		self.yaw_error=0.0

		#previous error
		self.throt_prev_error=0.0
		self.pitch_prev_error=0.0
		self.roll_prev_error=0.0
		self.yaw_prev_error=0.0

		#sum of error
		self.throt_error_sum=0.0
		self.pitch_error_sum=0.0
		self.roll_error_sum=0.0
		self.yaw_error_sum=0.0

		self.roll_i=0
		self.pitch_i=0
		self.throt_i=0
		self.yaw_i=0
		self.i=0

		# Position to hold.
		self.wp_x = np.array([0.0,4.21,0.0])
		self.wp_y = np.array([0.0,-2.13,0.0])#np.array([0.0,0.2,0.0])
		self.wp_z = 15.0
		self.wp_o=160.0
		
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000
		self.cmd.plutoIndex = 0
		
		#variables for drone location
		self.drone_x = 0.0
		self.drone_y = 0.0
		self.drone_z = 0.0
		self.drone_o=0.0

		#PID constants for Roll
		self.kp_roll =
		self.ki_roll =
		self.kd_roll =

		#PID constants for Pitch
		self.kp_pitch = 
		self.ki_pitch = 
		self.kd_pitch = 
		#PID constants for Yaw
		self.kp_yaw = 
		self.ki_yaw = 
		self.kd_yaw = 

		#PID constants for Throttle
		self.kp_throt = 
		self.ki_throt = 
		self.kd_throt = 

		# Correction values after PID is computed
		self.correct_roll = 0.0
		self.correct_pitch = 0.0
		self.correct_yaw = 0.0
		self.correct_throt = 0.0

		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.023

		rospy.sleep(.1)

	#for arming the drone put self.cmd.rcAux4=1500 and self.cmd.rcThrottle=1000 
	def arm(self):
		self.cmd.rcAUX4 = 1500
		self.cmd.rcThrottle = 1000
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.2)

	#for disarming the drone put self.cmd.rcAux4=1100 
	
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)

	#for lifting the drone put self.cmd.rcAux4=1500 and self.cmd.rcThrottle=1700 
		
	def takeoff(self):
		self.cmd.rcThrottle=1500
		self.cmd.rcAUX4=1500
		self.cmd.rcAUX3=0
		self.cmd.rcAUX2=0
		self.cmd.rcAUX1=0
		self.cmd.rcPitch=1500
		self.cmd.rcRoll=1500
		self.cmd.rcYaw=1500
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.2)

	#this is the function that publishes the command on drone in the topic /drone_command after calculating the PID
	
	def position_hold(self):

		rospy.sleep(2)

		print "disarm"
		self.disarm()
		rospy.sleep(.2)
		print "arm"
		self.arm()
		rospy.sleep(.1)
		print "takeoff"
		#self.takeoff()
		rospy.sleep(.2)
		
		while True:

			self.calc_pid()

			if self.i==0:
				#self.calc_pid()
				if((self.r_err>-2 and self.r_err<2)and(self.p_err>-2 and self.p_err<2)):
					if(self.throt_error>-2 and self.throt_error<2):
						self.i=self.i+1
						print self.i
			
			#adjusting the pitch value after calculating PID
		 	pitch_value = int(1500 + self.correct_pitch)
			#setting the rcPitch as the output of self.limit function
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
			#print self.cmd.rcPitch
			
			#adjusting the roll value after calculating PID 											
			roll_value = int(1500 + self.correct_roll)
			#setting the rcRoll as the output of self.limit function
			self.cmd.rcRoll = self.limit(roll_value, 1650,1450)
			
			#adjusting the roll value after calculating PID 												
			throt_value = int(1500 - self.correct_throt)
			#setting the rcThrottle as the output of self.limit function
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)

			yaw_value=int(1500 - self.correct_yaw)
			self.cmd.rcYaw=self.limit(yaw_value,1550,1450)
			#print self.cmd.rcYaw
			#publishing the commmands to drone								
			self.pluto_cmd.publish(self.cmd)
	
	#This function calls the all PID functions after a specific loop time
	
	def calc_pid(self):
		self.seconds = time.time()
		current_time = self.seconds - self.last_time
		if(current_time >= self.loop_time):
			self.pid_roll() #calling pid_roll
			self.pid_pitch()#calling pid_pitch
			self.pid_throt()#calling pid_throt
			self.pid_yaw()#calling pid_yaw
			
			self.last_time = self.seconds
	
#This function calculates the PID of roll for the values of self.kp_roll, self.kd_roll and self.ki_roll
	
	def pid_roll(self):
		
                self.roll_time=time.time()
		#calculates the dt 
                time_change=self.roll_time-self.roll_lt
		#error is give in self.wp_y-self.drone_y
                self.r_err=self.wp_y[self.i]-self.drone_y
                if self.roll_i!=0:
                        self.roll_error_sum+=(self.r_err*time_change)
                #Calculates the correct value of roll using PID equation
                self.correct_roll=(self.kp_roll*self.r_err)+(self.kd_roll*((self.r_err-self.roll_prev_error)/time_change))+(self.ki_roll*(self.roll_error_sum))
          
                self.roll_prev_error=self.r_err
                self.roll_lt=self.roll_time
                self.roll_i+=1

	def pid_pitch(self):

		self.pitch_time=time.time()
		#calculates the dt 
		time_change=self.pitch_time-self.pitch_lt
		#error is give in self.wp_x-self.drone_x
                self.p_err=self.wp_x[self.i]-self.drone_x
                if self.pitch_i!=0:
                        self.pitch_error_sum+=(self.p_err*time_change)
                #Calculates the correct value of pitch using PID equation
                self.correct_pitch=(self.kp_pitch*self.p_err)+(self.kd_pitch*((self.p_err-self.pitch_prev_error)/time_change))+(self.ki_pitch*(self.pitch_error_sum))
                
                self.pitch_prev_error=self.p_err
                self.pitch_lt=self.pitch_time
                self.roll_i+=1

	#This function calculates the PID of throttle for the values of self.kp_throt, self.kd_throt and self.ki_throt

	def pid_throt(self):

                self.throt_time=time.time()
		#calculates the dt
                time_change=self.throt_time-self.throt_lt
		#error is give in self.wp_z-self.drone_z
                self.throt_error=self.wp_z-self.drone_z
               
                        
                if self.throt_i!=0:
                        self.throt_error_sum+=(self.throt_error*time_change)
                #Calculates the correct value of throttle using PID equation
                self.correct_throt=(self.kp_throt*self.throt_error)+(self.kd_throt*((self.throt_error-self.throt_prev_error)/time_change))+(self.ki_throt*(self.throt_error_sum))
		
                self.throt_prev_error=self.throt_error
                self.throt_lt=self.throt_time
                self.throt_i+=1

	#This function calculates the PID of yaw for the values of self.kp_yaw, self.kd_yaw and self.ki_yaw
	
        def pid_yaw(self):
                
                self.yaw_time=time.time()
		#calculates the dt
                time_change=self.yaw_time-self.yaw_lt
		#error is give in self.wp_o-self.drone_o
                self.yaw_error=self.wp_o-self.drone_o
                if self.yaw_i!=0:
                        self.yaw_error_sum+=(self.yaw_error*time_change)
		##Calculates the correct value of yaw using PID equation
                self.correct_yaw=(self.kp_yaw*self.yaw_error)+(self.kd_yaw*((self.yaw_error-self.yaw_prev_error)/time_change))+(self.ki_yaw*(self.yaw_error_sum))
		
                self.yaw_prev_error=self.yaw_error
                self.yaw_lt=self.yaw_time
                self.yaw_i+=1

	#Function Name:limit()
	#Input:__
	#output:__
	#Logic:This function is used for limiting tha MAX and MIN values of command to be given
	#example call:self.limit()


	def limit(self, input_value, max_value, min_value):

		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value
	#This function is used for getting the position of drone 
		
	def get_pose(self,pose):

		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z
	
	def get_yaw(self,yaw):
                self.drone_o=yaw.data

	#def red(self, red):
		#self.red=red.data
		#print self.red
	
if __name__ == '__main__':
	while not rospy.is_shutdown():
		temp = DroneFly()
		temp.position_hold()
		rospy.spin()
