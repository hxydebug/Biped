#!/usr/bin/env python3
import rospy 
import numpy as np
import math
from std_msgs.msg import Float32, Int8, Float32MultiArray
import time 
from sensor_msgs.msg import Imu
from datetime import datetime

from matplotlib import pyplot as plt
import pandas as pd
import numpy as np



# define variable for subscriber
PI = math.pi
pi  = PI
D2R = PI / 180.0
R2D = 180.0 / PI


'''global value'''
global Leg_Origin
global Leg_Link_Length
global Leg1
global leg1_bias
global leg2_bias
global Leg1_Joint1_limt
global Leg1_Joint2_limt
global Leg1_Joint3_limt
global Joint_Velocity_Limit 
global Safe 
global Danger
global Cmd_violate
global Encoder_Wrong
global Large_Error
global angle_x
global angle_z

angle_x = 0
angle_z = 0

Cmd_violate = 1
Encoder_Wrong = 2
Large_Error = 3
Large_Velo = 4

Safe = 1
Danger = 0

'''leg property and limit''' 
leg1_bias = [2.674, 2.593,  1.775+(PI-2.6096)]
leg2_bias = [-2.931, -3.133, 0.834-(PI-2.6096)]

Leg1_Joint1_limt = [leg1_bias[0]-0.7, leg1_bias[0]+0.3]
Leg1_Joint2_limt = [leg1_bias[1]-0.9, leg1_bias[1]+0.5]
Leg1_Joint3_limt = [leg1_bias[2]-0.5, leg1_bias[2]+0.1]

Leg2_Joint1_limt = [leg2_bias[0]-0.7, leg2_bias[0]+0.3]
Leg2_Joint2_limt = [leg2_bias[1]-0.9, leg2_bias[1]+0.5]
Leg2_Joint3_limt = [leg2_bias[2]-0.5, leg2_bias[2]+0.1]

Joint_Velocity_Limit = PI/6.0

Leg_Origin = [0.0, 0.0, 0.0]
# L1 = 0.0873 L2 = 0.2191 L3 = 0.2350
Leg_Link_Length = [0.08915, 0.2202, 0.248158]
	
Flag_leg1 = 1
Flag_leg2 = 2 

swing_leg = 0
stance_leg = 1

nominal_stance_duration = [0.3,0.3]
nominal_stance_dutyrate = [0.5, 0.5]
# define gait class
class Gait:
	def __init__(self):

		self.stance_duration = nominal_stance_duration
		self.stance_dutyrate = nominal_stance_dutyrate

		self.initial_leg_state = [swing_leg,stance_leg]
		self.initial_leg_phase = [0,0]
		self.initial_state_ratio_in_cycle = [0,0]
		self.next_leg_state = [0,0]

		self.desired_leg_state = [0,0]
		self.leg_state = [0,0]
		self.normalized_phase = [0,0]

		for i in range(2):
			if self.initial_leg_state[i] == swing_leg:
				self.initial_state_ratio_in_cycle[i] = 1-self.stance_dutyrate[i]
            	self.next_leg_state[i] = stance_leg
			else:
				self.initial_state_ratio_in_cycle[i] = self.stance_dutyrate[i]
            	self.next_leg_state[i] = swing_leg

	def update(self, current_time):

		for leg_id in range(2):

			full_cycle_period = self.stance_duration[leg_id] / self.stance_dutyrate[leg_id]
			augmented_time = current_time + self.initial_leg_phase[leg_id] * full_cycle_period
			phase_in_full_cycle = math.fmod(augmented_time,full_cycle_period) / full_cycle_period
            ratio = self.initial_state_ratio_in_cycle[leg_id]

			if phase_in_full_cycle < ratio:
				self.desired_leg_state[leg_id] = self.initial_leg_state[leg_id]
				self.normalized_phase[leg_id] = phase_in_full_cycle / ratio
			else:
				# A phase switch happens for this leg.
				self.desired_leg_state[leg_id] = self.next_leg_state[leg_id]
				self.normalized_phase[leg_id] = (phase_in_full_cycle - ratio) / (1 - ratio)

			self.leg_state[leg_id] = self.desired_leg_state[leg_id]

# define swing_controller class
class swing_controller:
	def __init__(self,leg,gait):

		self.leg = leg
		self.gait = gait
		
		self.last_leg_state = self.gait.

# define joint class
class Joint:
	def __init__(self, position=0.0, velocity=0.0, torque=0.0, kp=0.0, kd=0.0):
		self.Position = position
		self.Velocity = velocity
		self.Torque = torque
		self.Kp = kp
		self.Kd = kd

# define the leg class cointains three joints and relative function 
class Leg:
	def __init__(self, flag, rospub):
		self.Joint1_cmd = Joint()
		self.Joint2_cmd = Joint()
		self.Joint3_cmd = Joint()

		self.Joint1_fb = Joint()
		self.Joint2_fb = Joint()
		self.Joint3_fb = Joint()

		self.flag = flag
		self.rospub = rospub
		self.foot_position = None
		self.joint_position = None
		self.cmd_array = None
		self.Safety_Flag = Safe
		self.Danger_info = 0

		self.L0 = Leg_Link_Length[0]
		self.L1 = Leg_Link_Length[1]
		self.L2 = Leg_Link_Length[2]

	def forward_kine(self, Q):

		c_0 = math.cos(Q[0])
		s_0 = math.sin(Q[0])
		c_1 = math.cos(Q[1])
		s_1 = math.sin(Q[1])
		c_2 = math.cos(Q[2])
		s_2 = math.sin(Q[2])

		if self.flag == 1:
			foot_x =  self.L0*s_0 + self.L2*(c_0*c_1*s_2 + c_0*c_2*s_1) + self.L1*c_0*s_1
			foot_y = -self.L0*c_0 + self.L2*(c_1*s_0*s_2 + c_2*s_0*s_1) + self.L1*s_0*s_1
			foot_z = - self.L2*(c_1*c_2 - s_1*s_2) - self.L1*c_1

			self.foot_position = [foot_x, foot_y, foot_z]

		elif self.flag == 2:
			foot_x =  - self.L0*s_0 - self.L2*(c_0*c_1*s_2 + c_0*c_2*s_1) - self.L1*c_0*s_1
			foot_y = self.L0*c_0 - self.L2*(c_1*s_0*s_2 + c_2*s_0*s_1) - self.L1*s_0*s_1
			foot_z = - self.L2*(c_1*c_2 - s_1*s_2) - self.L1*c_1

			self.foot_position = [foot_x, foot_y, foot_z]

		return self.foot_position

	def inverse_kine(self, P):

		x = P[0]
		y = P[1]
		z = P[2]

		if self.flag == 1:
			q2 = -math.acos((x*x + y*y + z*z - self.L0**2 - self.L1**2 - self.L2**2)/(2*self.L1*self.L2))
			q0 = 2*math.atan((x-(x*x+y*y-self.L0*self.L0)**0.5)/(-y+self.L0))
			a = -self.L2*math.cos(q2)-self.L1
			b = self.L2*math.sin(q2)
			c = z
			q1 = 2*math.atan((b-(b*b+a*a-c*c)**0.5)/(a+c))
			self.foot_position = [foot_x, foot_y, foot_z]

		elif self.flag == 2:
			q2 = math.acos((x*x + y*y + z*z - self.L0**2 - self.L1**2 - self.L2**2)/(2*self.L1*self.L2))
			q0 = 2*math.atan((-x-(x*x+y*y-self.L0*self.L0)**0.5)/(y+self.L0))
			a = -self.L2*math.cos(q2)-self.L1
			b = self.L2*math.sin(q2)
			c = z
			q1 = 2*math.atan((b-(b*b+a*a-c*c)**0.5)/(a+c))
			self.joint_position = [q0, q1, q2]

		return self.joint_position
	
	
	def cmd_compose(self):
		if self.flag == 1:  # right leg
			self.cmd_array = [
				-self.Joint1_cmd.Position + leg1_bias[0], -self.Joint1_cmd.Velocity, -self.Joint1_cmd.Torque, self.Joint1_cmd.Kp, self.Joint1_cmd.Kd,
				 self.Joint2_cmd.Position + leg1_bias[1], self.Joint2_cmd.Velocity, self.Joint2_cmd.Torque, self.Joint2_cmd.Kp, self.Joint2_cmd.Kd,
				 self.Joint3_cmd.Position + leg1_bias[2], self.Joint3_cmd.Velocity, self.Joint3_cmd.Torque, self.Joint3_cmd.Kp, self.Joint3_cmd.Kd
			]

			print(self.cmd_array)
			
			'''safety check for command if winthin the range before send to publisher'''
			if (	self.cmd_array[0] < Leg1_Joint1_limt[0] or self.cmd_array[0] > Leg1_Joint1_limt[1]
				or 	self.cmd_array[5] < Leg1_Joint2_limt[0] or self.cmd_array[5] > Leg1_Joint2_limt[1]
				or 	self.cmd_array[10]< Leg1_Joint3_limt[0] or self.cmd_array[10] > Leg1_Joint3_limt[1]):
				self.Safety_Flag = Danger
				self.Danger_info = Cmd_violate
				# print(Leg1_Joint2_limt[0]-self.Joint2_cmd.Position)
				# print(Leg1_Joint2_limt[1]-self.Joint2_cmd.Position)
				# print(Leg1_Joint3_limt[0]-self.Joint3_cmd.Position)
				# print(Leg1_Joint3_limt[1]-self.Joint3_cmd.Position)
				self.safety_and_reset()

			if (abs(self.Joint1_cmd.Position-self.Joint1_fb.Position)>PI/6.0 or abs(self.Joint3_cmd.Position-self.Joint3_fb.Position)>PI/6.0):
				# or abs(self.Joint2_cmd.Position-self.Joint2_fb.Position)>PI/6.0 
				self.Danger_info = Large_Error
				self.Safety_Flag = Danger
				self.safety_and_reset()
			

		elif self.flag == 2:  # left leg
			self.cmd_array = [
				-self.Joint1_cmd.Position + leg2_bias[0], -self.Joint1_cmd.Velocity, -self.Joint1_cmd.Torque, self.Joint1_cmd.Kp, self.Joint1_cmd.Kd,
				-self.Joint2_cmd.Position + leg2_bias[1], -self.Joint2_cmd.Velocity, -self.Joint2_cmd.Torque, self.Joint2_cmd.Kp, self.Joint2_cmd.Kd,
				-self.Joint3_cmd.Position + leg2_bias[2], -self.Joint3_cmd.Velocity, -self.Joint3_cmd.Torque, self.Joint3_cmd.Kp, self.Joint3_cmd.Kd
			]
			print(self.cmd_array)

			'''safety check for command if winthin the range before send to publisher'''
			if (	self.cmd_array[0] < Leg2_Joint1_limt[0] or self.cmd_array[0] > Leg2_Joint1_limt[1]
				or 	self.cmd_array[5] < Leg2_Joint2_limt[0] or self.cmd_array[5] > Leg2_Joint2_limt[1]
				or 	self.cmd_array[10]< Leg2_Joint3_limt[0] or self.cmd_array[10] > Leg2_Joint3_limt[1]):
				self.Safety_Flag = Danger
				self.Danger_info = Cmd_violate
				# # print(Leg1_Joint2_limt[0]-self.Joint2_cmd.Position)
				# # print(Leg1_Joint2_limt[1]-self.Joint2_cmd.Position)
				# # print(Leg1_Joint3_limt[0]-self.Joint3_cmd.Position)
				# # print(Leg1_Joint3_limt[1]-self.Joint3_cmd.Position)
				self.safety_and_reset()
				pass

			if (abs(self.Joint1_cmd.Position-self.Joint1_fb.Position)>PI/6.0 or abs(self.Joint3_cmd.Position-self.Joint3_fb.Position)>PI/6.0 or abs(self.Joint2_cmd.Position-self.Joint2_fb.Position)>PI/6.0 ):
				if abs(self.Joint1_cmd.Position-self.Joint1_fb.Position)>PI/6.0:
					print('error joint1, fb_position:', self.Joint1_fb.Position)
				elif abs(self.Joint2_cmd.Position-self.Joint2_fb.Position)>PI/6.0:
					print('error joint2, fb_position:', self.Joint2_fb.Position)
				elif abs(self.Joint3_cmd.Position-self.Joint3_fb.Position)>PI/6.0:
					print('error joint3, fb_position:', self.Joint3_fb.Position)
				# or abs(self.Joint2_cmd.Position-self.Joint2_fb.Position)>PI/6.0 
				self.Danger_info = Large_Error
				self.Safety_Flag = Danger
				self.safety_and_reset()

		return self.cmd_array
	
	def control(self):
		Leg_pub_cmd = Float32MultiArray()
		Leg_pub_cmd.data = self.cmd_compose() # check position limit before pub
		if self.Safety_Flag == Safe: # do not remove 
			self.rospub.publish(Leg_pub_cmd)				

	def safety_and_reset(self):
		print('error...')
		reset_cmd = Float32MultiArray()
		self.cmd_array = [0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0]
		reset_cmd.data = self.cmd_array
		self.rospub.publish(reset_cmd)
		time.sleep(0.01)

		if (self.Danger_info == Cmd_violate):				
			print("Position cmd violates limit!")
		elif (self.Danger_info == Encoder_Wrong):
			print("Position fb violates limit, encoder reading WRONG!")
		elif (self.Danger_info == Large_Error):
			print("Large Angle Error Detected!")
		elif (self.Danger_info == Large_Velo):
			print("Large Angle Velo Detected!")
		print("RESET! Leg " + str(self.flag) + ": Danger!")


	def hold_current_position(self):
		self.Joint1_cmd.Position = self.Joint1_fb.Position
		self.Joint1_cmd.Velocity = 0
		self.Joint1_cmd.Torque =  0
		self.Joint1_cmd.Kp = 50
		self.Joint1_cmd.Kd = 15

		self.Joint2_cmd.Position = self.Joint2_fb.Position
		self.Joint2_cmd.Velocity = 0
		self.Joint2_cmd.Torque = 0
		self.Joint2_cmd.Kp = 50
		self.Joint2_cmd.Kd = 25

		self.Joint3_cmd.Position = self.Joint3_fb.Position
		self.Joint3_cmd.Velocity = 0
		self.Joint3_cmd.Torque = 0
		self.Joint3_cmd.Kp = 35
		self.Joint3_cmd.Kd = 15
		self.control()
	




	
# define callback function 

def Leg1_Cb(data):
	global Leg1
	Leg1.Joint1_fb = Joint(-(data.data[0] - leg1_bias[0]), -data.data[1], -data.data[2])
	Leg1.Joint2_fb = Joint(  data.data[3] - leg1_bias[1],   data.data[4],  data.data[5])
	Leg1.Joint3_fb = Joint(  data.data[6] - leg1_bias[2],   data.data[7],  data.data[8])

	# '''check jonit angle if within the limit'''
	# if(	data.data[0] < Leg1_Joint1_limt[0] or data.data[0] > Leg1_Joint1_limt[1]
	# 	or data.data[3] < Leg1_Joint2_limt[0] or data.data[3] > Leg1_Joint2_limt[1]):
		
	# 	Leg1.Safety_Flag = Danger
	# 	Leg1.Danger_info = Encoder_Wrong
	# 	Leg1.safety_and_reset()	
	# if(	data.data[0] < Leg1_Joint1_limt[0] or data.data[0] > Leg1_Joint1_limt[1]):
	# 	print('encoder 1 error')
	# if(	data.data[3] < Leg1_Joint2_limt[0] or data.data[3] > Leg1_Joint2_limt[1]):
	# 	print('encoder 2 error')
	# if(	data.data[6] < Leg1_Joint3_limt[0] or data.data[6] > Leg1_Joint3_limt[1]):
	# 	print(data.data)
	# 	print('encoder 3 error')
	# '''check the joint angular velocity within the limit PI/6'''		
	# if (abs(Leg1.Joint1_cmd.Velocity ) > Joint_Velocity_Limit): 
	# 	Leg1.Danger_info = Large_Velo
	# 	Leg1.Safety_Flag = Danger
	# 	Leg1.safety_and_reset()
		



def Leg2_Cb(data):
	global Leg2
	Leg2.Joint1_fb = Joint(-(data.data[0] - leg2_bias[0]), -data.data[1], -data.data[2])
	Leg2.Joint2_fb = Joint(-(data.data[3] - leg2_bias[1]), -data.data[4], -data.data[5])
	Leg2.Joint3_fb = Joint(-(data.data[6] - leg2_bias[2]), -data.data[7], -data.data[8])
	if(data.data[0] < Leg2_Joint1_limt[0] or data.data[0] > Leg2_Joint1_limt[1]
		or data.data[3] < Leg2_Joint2_limt[0] and data.data[3] > Leg2_Joint2_limt[1]
		or data.data[6] < Leg2_Joint3_limt[0] and data.data[6] > Leg2_Joint3_limt[1]):
		Leg2.Danger_info = Encoder_Wrong
		Leg2.Safety_Flag = Danger
		if(	data.data[0] < Leg2_Joint1_limt[0] or data.data[0] > Leg2_Joint1_limt[1]):
			print('encoder 1 error')
		if(	data.data[3] < Leg2_Joint2_limt[0] or data.data[3] > Leg2_Joint2_limt[1]):
			print('encoder 2 error')
		if(	data.data[6] < Leg2_Joint3_limt[0] or data.data[6] > Leg2_Joint3_limt[1]):
			print(data.data)
			print('encoder 3 error')

		'''check the joint angular velocity within the limit PI/6'''		
	if (abs(Leg2.Joint1_cmd.Velocity ) > Joint_Velocity_Limit or abs(Leg2.Joint2_cmd.Velocity) > Joint_Velocity_Limit  
		or abs(Leg2.Joint3_cmd.Velocity) > Joint_Velocity_Limit): 
		Leg2.Danger_info = Large_Velo
		Leg2.Safety_Flag = Danger
		Leg2.safety_and_reset()


	

if __name__ == '__main__':
	rospy.init_node('Test')

	# publisch command
	Leg1_pub = rospy.Publisher('/leg/leg1/cmd', Float32MultiArray, queue_size = 2)
	Leg2_pub = rospy.Publisher('/leg/leg2/cmd', Float32MultiArray, queue_size = 2)


	'''!!!!!!!!! Danger---Danger---Danger---Danger---Danger---Danger!!!!!!!!!'''

	Leg1 = Leg(Flag_leg1, Leg1_pub)	
	if 'Leg1' not in globals(): # if leg1 not defined 
		rospy.signal_shutdown("Stopped! Leg1 not defined before Subscriber")
		print("Rospy Stopped! Leg1 not defined before Subscriber!")
		Leg1 = Leg(Flag_leg1, Leg1_pub)	
		print("Leg1 Created! However You MUST creat it yourself: \n Leg1 = Leg(Flag_leg1, Leg1_pub)")

	Leg2= Leg(Flag_leg2, Leg2_pub)	
	if 'Leg2' not in globals(): # if leg2 not defined 
		rospy.signal_shutdown("Stopped! Leg2 not defined before Subscriber")
		print("Stopped! Leg2 not defined before Subscriber")
		Leg2 = Leg(Flag_leg2, Leg2_pub)
		print("Leg2 Created!")

	'''!!!!!!!!! Danger---Danger---Danger---Danger---Danger---Danger!!!!!!!!!'''

	# feedback
	rospy.Subscriber('/leg/leg1/feedback', Float32MultiArray, Leg1_Cb)
	rospy.Subscriber('/leg/leg2/feedback', Float32MultiArray, Leg2_Cb)
	
	Frequency = 500
	dt = 1.0/Frequency
	rate = rospy.Rate(Frequency)

	'''print data to file'''
	File_name = time.strftime('%Y_%m_%d_%H_%M_%S')
	file = open('./Data/Exp_' + File_name + '.txt', 'w')
	time.sleep (1.5)
	

	print('----------Rospy Loop Started----------')
	T0 = rospy.get_time ()
	while not rospy.is_shutdown():
		t = rospy.get_time() - T0
		
		'''publish the planned trajectory one point by one point'''



		''' pd and torque set '''
		
		Leg1.Joint1_cmd.Torque = 0.0# 0
		Leg1.Joint1_cmd.Kp = 80.0# 110.0
		Leg1.Joint1_cmd.Kd = 6.0# 6.0
		Leg1.Joint2_cmd.Torque = 0.0# 0
		Leg1.Joint2_cmd.Kp = 80.0# 110.0
		Leg1.Joint2_cmd.Kd = 6.0# 6.0
		Leg1.Joint3_cmd.Torque = 0.0# 0
		Leg1.Joint3_cmd.Kp = 80.0# 110.0
		Leg1.Joint3_cmd.Kd = 6.0# 6.0

		Leg2.Joint1_cmd.Torque = 0.0# 0
		Leg2.Joint1_cmd.Kp = 80.0# 110.0
		Leg2.Joint1_cmd.Kd = 6.0# 6.0
		Leg2.Joint2_cmd.Torque = 0.0# 0
		Leg2.Joint2_cmd.Kp = 80.0# 110.0
		Leg2.Joint2_cmd.Kd = 6.0# 6.0
		Leg2.Joint3_cmd.Torque = 0.0# 0
		Leg2.Joint3_cmd.Kp = 80.0# 110.0
		Leg2.Joint3_cmd.Kd = 6.0# 6.0
		

		'''check encoder reading and command sending are within the joint angle range
		otherwise stop the leg'''
		# print(11111, Leg1.Joint1_fb.Position, Leg1.Joint2_fb.Position, Leg1.Joint3_fb.Position)
		# print(22222, Leg1.Joint1_cmd.Position, Leg1.Joint2_cmd.Velocity, Leg1.Joint3_cmd.Position)
		
		if Leg1.Safety_Flag == Danger: 
			Leg1.safety_and_reset()
			rospy.signal_shutdown("Stopped! Error Occured!")
			print("----------Rospy Loop Stopped!----------")
			print("Leg_cmd: ", Leg1.cmd_array[0],  Leg1.cmd_array[5],  Leg1.cmd_array[10])
			break
		elif Leg1.Safety_Flag == Safe:
			Leg1.control()
			
		if Leg2.Safety_Flag == Danger: 
			Leg2.safety_and_reset()
			rospy.signal_shutdown("Stopped! Error Occured!")
			print("----------Rospy Loop Stopped!----------")
			print("Leg_cmd: ", Leg2.cmd_array[0],  Leg2.cmd_array[5],  Leg2.cmd_array[10])
			break
		elif Leg2.Safety_Flag == Safe:
			Leg2.control()


		#print("feedback angle joint2: ", Leg2.Joint2_fb.Position+ leg2_bias[1])
		#print("feedback angle joint3: ", Leg2.Joint3_fb.Position+ leg2_bias[2])

		# write data into txt file with 8 decimal digits
		file.write('{:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} \n'.format(t, Leg2.Joint1_fb.Position, Leg2.Joint1_fb.Velocity, Leg2.Joint1_fb.Torque,\
																																										Leg2.Joint2_fb.Position, Leg2.Joint2_fb.Velocity, Leg2.Joint2_fb.Torque, \
																																											Leg2.Joint3_fb.Position, Leg2.Joint3_fb.Velocity, Leg2.Joint3_fb.Torque,\
																																											Leg2.Joint1_cmd.Position, Leg2.Joint1_cmd.Velocity, Leg2.Joint1_cmd.Torque, \
																																												Leg2.Joint2_cmd.Position, Leg2.Joint2_cmd.Velocity, Leg2.Joint2_cmd.Torque,\
																																													Leg2.Joint3_cmd.Position, Leg2.Joint3_cmd.Velocity, Leg2.Joint3_cmd.Torque))
		rate.sleep()
