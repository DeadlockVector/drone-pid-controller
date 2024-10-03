#!/usr/bin/env python3

'''
This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/pid_error			/throttle_pid
						/pitch_pid
						/roll_pid
					
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node


class Swift_Pico(Node):
	def __init__(self):
		super().__init__('pico_controller')  # initializing ros node with name pico_controller

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0, 0.0, 0.0]

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [2, 2, 20]  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

		# Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = SwiftMsgs()
		self.cmd.rc_roll = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_throttle = 1500

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [2, 0.2, 5]                  
		self.Ki = [0.009, 0, 1]                     # 10/5, 0.4, 22 -> 0:30
		self.Kd = [0.1, 0, 20]                      #5, 1, 20 -> 0:15 to 0:20
		                                           #x:2,0.09,4


		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.pid_error = [0, 0, 0]
		self.previous_error = [0, 0, 0]
		self.integral_error = [0, 0, 0]
		self.derivative_error = [0, 0, 0]

		# add upper and lower limit here
		self.min_values = [1000, 1000, 1000]
		self.max_values = [2000, 2000, 2000]

		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit.
	
		#self.sample_time = 0.060  # in seconds
		self.sample_time = 1

		# Publishing /drone_command, /pid_error
		self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
		self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

		#------------------------Add other ROS 2 Publishers here-----------------------------------------------------
	

		# Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
		self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
		self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)

		#------------------------Add other ROS Subscribers here-----------------------------------------------------
		# /pitch_pid
		# /roll_pid
		self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)	
		self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)

		self.arm()  # ARMING THE DRONE

		# Creating a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(Python)
		self.create_timer(self.sample_time, self.pid)

	def disarm(self):
		self.cmd.rc_roll = 1000
		self.cmd.rc_yaw = 1000
		self.cmd.rc_pitch = 1000
		self.cmd.rc_throttle = 1000
		self.cmd.rc_aux4 = 1000
		self.command_pub.publish(self.cmd)
		

	def arm(self):
		self.disarm()
		self.cmd.rc_roll = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_throttle = 1500
		self.cmd.rc_aux4 = 2000
		self.command_pub.publish(self.cmd)  # Publishing /drone_command


	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self, msg):

		# check poses[0] or poses[1]
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z

		#print(str(self.drone_position)+"drone position")

		#---------------------------------------------------------------------------------------------------------------


	# Callback function for /throttle_pid
	# This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
	def altitude_set_pid(self, alt):
		self.Kp[2] = alt.kp * 0.03    # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.ki * 0.008
		self.Kd[2] = alt.kd * 0.6

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	#----------------------------------------------------------------------------------------------------------------------#

	def pitch_set_pid(self, pitch):
		self.Kp[1] = pitch.kp*0.03
		self.Ki[1] = pitch.ki*0.008
		self.Kd[1] = pitch.kd*0.6
		

	def roll_set_pid(self, roll):
		self.Kp[0] = roll.kp*0.03
		self.Ki[0] = roll.ki*0.008
		self.Kd[0] = roll.kd*0.6



	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum

		# stuff
		self.pid_error = [
			self.setpoint[0]-self.drone_position[0], 
			self.setpoint[1]-self.drone_position[1], 
			self.setpoint[2]-self.drone_position[2]
		]

		#print(str(self.pid_error)+"pid_error")

		self.integral_error = [
			self.integral_error[0]+self.pid_error[0],
			self.integral_error[1]+self.pid_error[1],
			self.integral_error[2]+self.pid_error[2]
		] 

		self.derivative_error = [
			self.pid_error[0]-self.previous_error[0],
			self.pid_error[1]-self.previous_error[1],
			self.pid_error[2]-self.previous_error[2]
		]

		self.previous_error = self.pid_error
        
		self.pid_output = [
			self.Kp[0]*self.pid_error[0] + self.Ki[0]*self.integral_error[0] + self.Kd[0]*self.derivative_error[0],
			self.Kp[1]*self.pid_error[1] + self.Ki[1]*self.integral_error[1] + self.Kd[1]*self.derivative_error[1],
			self.Kp[2]*self.pid_error[2] + self.Ki[2]*self.integral_error[2] + self.Kd[2]*self.derivative_error[2]
		]

		'''
		print(str(self.Kp[2]*self.pid_error[2])+"pidkp")
		print(str(self.Ki[2]*self.integral_error[2])+"pidki")
		print(str(self.Kd[2]*self.derivative_error[2])+"pidkd")
		print(self.pid_output[2])
		print(self.Kp[2]*self.pid_error[2] + self.Ki[2]*self.integral_error[2] + self.Kd[2]*self.derivative_error[2])
		'''

		# pid error message of type PIDError
		self.pid_error_msg = PIDError()
		self.pid_error_msg.roll_error = self.pid_error[0]
		self.pid_error_msg.pitch_error = self.pid_error[1]
		self.pid_error_msg.throttle_error = self.pid_error[2]
		self.pid_error_msg.yaw_error = 0.0
		
		self.cmd.rc_roll = self.getVal(self.min_values[0], self.max_values[0], self.pid_output[0]+1500)
		self.cmd.rc_pitch = self.getVal(self.min_values[1], self.max_values[1], 1500-self.pid_output[1])
		self.cmd.rc_throttle = self.getVal(self.min_values[2], self.max_values[2], 1500-self.pid_output[2])
		self.cmd.rc_yaw = 1500

		'''
		self.cmd.rc_roll = min(1500 + int(self.pid_output[0]), self.max_values[0])
		self.cmd.rc_pitch = min(1500 + int(self.pid_output[1]), self.max_values[1])
		self.cmd.rc_throttle = min(1500 - int(self.pid_output[2]), self.max_values[2])
		self.cmd.rc_yaw = 1500

		self.cmd.rc_roll = self.max_values[0] if (self.pid_output[0]+1500) > self.max_values[0] else (self.min_values[0] if (self.pid_output[0]+1500) < self.min_values[0] else int(self.pid_output[0]+1500))
		self.cmd.rc_pitch = self.max_values[1] if (self.pid_output[1]+1500) > self.max_values[1] else (self.min_values[1] if (self.pid_output[1]+1500) < self.min_values[1] else int(self.pid_output[1]+1500))
		self.cmd.rc_throttle = self.max_values[2] if (self.pid_output[2]-1500) > self.max_values[2] else (self.min_values[2] if (self.pid_output[2]-1500) < self.min_values[2] else int(self.pid_output[2]-1500))
		self.cmd.rc_yaw = 1500

		self.cmd.rc_roll = 1500 + int(self.pid_output[0])
		self.cmd.rc_yaw = 1500
		self.cmd.rc_pitch = 1500 + int(self.pid_output[1])
		self.cmd.rc_throttle = 1500 - int(self.pid_output[2])
		'''
	#------------------------------------------------------------------------------------------------------------------------
		self.command_pub.publish(self.cmd)
		# calculate throttle error, pitch error and roll error, then publish it accordingly
		self.pid_error_pub.publish(self.pid_error_msg)

		#print(str(self.Kp)+"kp")
		#print(str(self.Ki)+"ki")
		#print(str(self.Kd)+"kd")
		
	def getVal(self, minVal, maxVal, val):
		if val<minVal:
			val = minVal
		elif val>maxVal:
			val = maxVal
		return int(val)



def main(args=None):
	rclpy.init(args=args)
	swift_pico = Swift_Pico()
	rclpy.spin(swift_pico)
	swift_pico.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
