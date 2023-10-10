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
L = 1.2


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
leg1_bias = [2.674, 2.593,  1.775]
leg2_bias = [-2.931, -3.133, 0.834]

Leg1_Joint1_limt = [leg1_bias[0]-0.7, leg1_bias[0]+0.3]
Leg1_Joint2_limt = [leg1_bias[1]-0.9, leg1_bias[1]+0.5]
Leg1_Joint3_limt = [leg1_bias[2]-0.5, leg1_bias[2]+0.1]

Leg2_Joint1_limt = [leg2_bias[0]-0.7, leg2_bias[0]+0.3]
Leg2_Joint2_limt = [leg2_bias[1]-0.9, leg2_bias[1]+0.5]
Leg2_Joint3_limt = [leg2_bias[2]-0.5, leg2_bias[2]+0.1]

Joint_Velocity_Limit = PI/6.0

Leg_Origin = [0.0, 0.0, 0.0]
# L1 = 0.0873 L2 = 0.2191 L3 = 0.2350
Leg_Link_Length = [0.0873, 0.2191, 0.2350]
	
Flag_leg1 = 1
Flag_leg2 = 2 


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
		self.cmd_array = None
		self.Safety_Flag = Safe
		self.Danger_info = 0
		self.x0 = Leg_Origin[0]
		self.y0 = Leg_Origin[1]
		self.z0 = Leg_Origin[2]
		self.L1 = Leg_Link_Length[0]
		self.L2 = Leg_Link_Length[1]
		self.L3 = Leg_Link_Length[2]

	def forward_kine(self, angle_x, Q=None):
		if Q is None:
			c_1 = math.cos(self.Joint1_fb.Position)
			s_1 = math.sin(self.Joint1_fb.Position)
			c_2 = math.cos(self.Joint2_fb.Position)
			s_2 = math.sin(self.Joint2_fb.Position)
			c_3 = math.cos(self.Joint3_fb.Position)
			s_3 = math.sin(self.Joint3_fb.Position)
		else:
			c_1 = math.cos(Q[0])
			s_1 = math.sin(Q[0])
			c_2 = math.cos(Q[1])
			s_2 = math.sin(Q[1])
			c_3 = math.cos(Q[2])
			s_3 = math.sin(Q[2])
		
		c_ang_x = math.cos(angle_x)
		s_ang_x = math.sin(angle_x)

		if self.flag == 1:
			foot_x = self.L3 * (c_1 * c_2 * s_3 + c_1 * c_3 * s_2) - self.L1 * s_1 + self.L2 * c_1 * c_2
			foot_y = self.L3 * (c_3 * (c_2 * s_ang_x + c_ang_x * s_1 * s_2) - s_3 * (s_ang_x * s_2 - c_ang_x * c_2 * s_1)) - self.L2 * (s_ang_x * s_2 - c_ang_x * c_2 * s_1) + self.y0 * c_ang_x - self.z0 * s_ang_x + self.L1 * c_ang_x * c_1
			foot_z = self.L2 * (c_ang_x * s_2 + c_2 * s_ang_x * s_1) - self.L3 * (c_3 * (c_ang_x * c_2 - s_ang_x * s_1 * s_2) - s_3 * (c_ang_x * s_2 + c_2 * s_ang_x * s_1)) + self.z0 * c_ang_x + self.y0 * s_ang_x + self.L1 * c_1 * s_ang_x

			self.foot_position = [foot_x, foot_y, foot_z]

		elif self.flag == 2:
			foot_x = self.L1 * s_1 + self.L3 * (c_1 * c_2 * s_3 + c_1 * c_3 * s_2) + self.L2 * c_1 * c_2
			foot_y = self.L3 * (c_3 * (c_2 * s_ang_x + c_ang_x * s_1 * s_2) - s_3 * (s_ang_x * s_2 - c_ang_x * c_2 * s_1)) - self.L2 * (s_ang_x * s_2 - c_ang_x * c_2 * s_1) - self.y0 * c_ang_x - self.z0 * s_ang_x - self.L1 * c_ang_x * c_1
			foot_z = self.L2 * (c_ang_x * s_2 + c_2 * s_ang_x * s_1) - self.L3 * (c_3 * (c_ang_x * c_2 - s_ang_x * s_1 * s_2) - s_3 * (c_ang_x * s_2 + c_2 * s_ang_x * s_1)) + self.z0 * c_ang_x - self.y0 * s_ang_x - self.L1 * c_1 * s_ang_x

			self.foot_position = [foot_x, foot_y, foot_z]

		return self.foot_position

	def Jocobian(self, angle_x, Q=None):
		if Q is None: # use foodback angle
			c_1 = math.cos(self.Joint1_fb.Position)
			s_1 = math.sin(self.Joint1_fb.Position)
			c_2 = math.cos(self.Joint2_fb.Position)
			s_2 = math.sin(self.Joint2_fb.Position)
			c_3 = math.cos(self.Joint3_fb.Position)
			s_3 = math.sin(self.Joint3_fb.Position)
		else: # user external angle
			c_1 = math.cos(Q[0])
			s_1 = math.sin(Q[0])
			c_2 = math.cos(Q[1])
			s_2 = math.sin(Q[1])
			c_3 = math.cos(Q[2])
			s_3 = math.sin(Q[2])           
		
		c_ang_x = math.cos(angle_x)
		s_ang_x = math.sin(angle_x)
		if self.flag == 1: # left leg
			RotMat = np.array([
				[ c_1 * c_2 * c_3 - c_1 * s_2 * s_3, -s_1, - c_1 * c_2 * s_3 - c_1 * c_3 * s_2],
				[ - c_3 * (s_ang_x * s_2 - c_ang_x * c_2 * s_1) - s_3 * (c_2 * s_ang_x + c_ang_x * s_1 * s_2), c_ang_x * c_1, s_3 * (s_ang_x * s_2 - c_ang_x * c_2 * s_1) - c_3 * (c_2 * s_ang_x + c_ang_x * s_1 * s_2)],
				[ c_3 * (c_ang_x * s_2 + c_2 * s_ang_x * s_1) + s_3 * (c_ang_x * c_2 - s_ang_x * s_1 * s_2), c_1 * s_ang_x, c_3 * (c_ang_x * c_2 - s_ang_x * s_1 * s_2) - s_3 * (c_ang_x * s_2 + c_2 * s_ang_x * s_1)]
			])

			Jac = np.array([
				[ - self.L3 * (c_2 * s_1 * s_3 + c_3 * s_1 * s_2) - self.L1 * c_1 - self.L2 * c_2 * s_1,  self.L3 * (c_1 * c_2 * c_3 - c_1 * s_2 * s_3) - self.L2 * c_1 * s_2, self.L3 * (c_1 * c_2 * c_3 - c_1 * s_2 * s_3)],
				[ self.L3 * (c_ang_x * c_1 * c_2 * s_3 + c_ang_x * c_1 * c_3 * s_2) - self.L1 * c_ang_x * s_1 + self.L2 * c_ang_x * c_1 * c_2, - self.L2 * (c_2 * s_ang_x + c_ang_x * s_1 * s_2) - self.L3 * (c_3 * (s_ang_x * s_2 - c_ang_x * c_2 * s_1) + s_3 * (c_2 * s_ang_x + c_ang_x * s_1 * s_2)), -self.L3 * (c_3 * (s_ang_x * s_2 - c_ang_x * c_2 * s_1) + s_3 * (c_2 * s_ang_x + c_ang_x * s_1 * s_2))],
				[ self.L3 * (c_1 * c_2 * s_ang_x * s_3 + c_1 * c_3 * s_ang_x * s_2) - self.L1 * s_ang_x * s_1 + self.L2 * c_1 * c_2 * s_ang_x,   self.L2 * (c_ang_x * c_2 - s_ang_x * s_1 * s_2) + self.L3 * (c_3 * (c_ang_x * s_2 + c_2 * s_ang_x * s_1) + s_3 * (c_ang_x * c_2 - s_ang_x * s_1 * s_2)),  self.L3 * (c_3 * (c_ang_x * s_2 + c_2 * s_ang_x * s_1) + s_3 * (c_ang_x * c_2 - s_ang_x * s_1 * s_2))]
			])

		elif self.flag == 2: # right leg
			RotMat = np.array([
					[  c_1 * c_2 * c_3 - c_1 * s_2 * s_3, -s_1, - c_1 * c_2 * s_3 - c_1 * c_3 * s_2],
					[ - c_3 * (s_ang_x * s_2 - c_ang_x * c_2 * s_1) - s_3 * (c_2 * s_ang_x + c_ang_x * s_1 * s_2), c_ang_x * c_1, s_3 * (s_ang_x * s_2 - c_ang_x * c_2 * s_1) - c_3 * (c_2 * s_ang_x + c_ang_x * s_1 * s_2)],
					[   c_3 * (c_ang_x * s_2 + c_2 * s_ang_x * s_1) + s_3 * (c_ang_x * c_2 - s_ang_x * s_1 * s_2), c_1 * s_ang_x, c_3 * (c_ang_x * c_2 - s_ang_x * s_1 * s_2) - s_3 * (c_ang_x * s_2 + c_2 * s_ang_x * s_1)]
			])
 
			Jac =np.array([
				[ self.L1 * c_1 - self.L3 * (c_2 * s_1 * s_3 + c_3 * s_1 * s_2) - self.L2 * c_2 * s_1,  self.L3 * (c_1 * c_2 * c_3 - c_1 * s_2 * s_3) - self.L2 * c_1 * s_2, self.L3 * (c_1 * c_2 * c_3 - c_1 * s_2 * s_3)],
				[ self.L3 * (c_ang_x * c_1 * c_2 * s_3 + c_ang_x * c_1 * c_3 * s_2) + self.L1 * c_ang_x * s_1 + self.L2 * c_ang_x * c_1 * c_2, - self.L2 * (c_2 * s_ang_x + c_ang_x * s_1 * s_2) - self.L3 * (c_3 * (s_ang_x * s_2 - c_ang_x * c_2 * s_1) + s_3 * (c_2 * s_ang_x + c_ang_x * s_1 * s_2)), -self.L3 * (c_3 * (s_ang_x * s_2 - c_ang_x * c_2 * s_1) + s_3 * (c_2 * s_ang_x + c_ang_x * s_1 * s_2))],
				[ self.L3 * (c_1 * c_2 * s_ang_x * s_3 + c_1 * c_3 * s_ang_x * s_2) + self.L1 * s_ang_x * s_1 + self.L2 * c_1 * c_2 * s_ang_x,   self.L2 * (c_ang_x * c_2 - s_ang_x * s_1 * s_2) + self.L3 * (c_3 * (c_ang_x * s_2 + c_2 * s_ang_x * s_1) + s_3 * (c_ang_x * c_2 - s_ang_x * s_1 * s_2)),  self.L3 * (c_3 * (c_ang_x * s_2 + c_2 * s_ang_x * s_1) + s_3 * (c_ang_x * c_2 - s_ang_x * s_1 * s_2))]
			])

		return Jac, RotMat

	def ground_reaction(self, angle_x):
		Joint_Tau = np.array([self.Joint1_fb.Torque, self.Joint2_fb.Torque, self.Joint3_fb.Torque])
		Jac, R = self.Jocobian(angle_x)

		Jac_T = np.transpose(Jac)
		R_T = np.transpose(R)

		foot_xyz = self.forward_kine(angle_x)
		Force_G = -(np.linalg.inv(Jac_T.dot(R_T))).dot(np.transpose(Joint_Tau)) # negative sign for ground reaction force 
		Torque_G = np.cross(foot_xyz, Force_G.tolist())  # cross produce for list only
		return Force_G.tolist(), Torque_G


	# def Inverse_Kine(self, foot_xyz_c, angle_x):
	# 	''' define the foot position error function'''

	# 	def Foot_Error_Fun(Q, *args):
	# 		angle_x = args[0]
	# 		foot_xyz_c = [args[1], args[2], args[3]]
	# 		foot_xyz = self.forward_kine(angle_x, Q)
	# 		return np.array(foot_xyz) - foot_xyz_c
		
	# 	Joint_Angle_0 = np.array([self.Joint1_fb.Position, self.Joint2_fb.Position,self.Joint3_fb.Position]) # use current joint angle as initial point 
	# 	Joint_Angle = optimize.fsolve(Foot_Error_Fun, Joint_Angle_0, args=(angle_x, foot_xyz_c[0], foot_xyz_c[1], foot_xyz_c[2]))
	# 	return Joint_Angle.tolist()
	
	def Traj_Plan(self, Joint_Angle_C, T, Joint_Angle_0=None):
		'''define Trajecotry function'''
		
		if Joint_Angle_0 is None: # user current joint angle as initial angle
			Q0 = [self.Joint1_fb.Position, self.Joint2_fb.Position, self.Joint3_fb.Position]
		else: # user external angle as initial angle
			Q0 = Joint_Angle_0
		
		Qd = Joint_Angle_C
		''' Use 5-order Biezer Polynominal for interpolation, parameters identified by optimization '''
		P =[0, 0.0009, -0.0041, 1.0000, 0.9986,1.0000]
		P0 = P[0]
		P1 = P[1]
		P2 = P[2]
		P3 = P[3]
		P4 = P[4]
		P5 = P[5]

		s = T/T[-1]
		B = P5*s**5 - P0*(s - 1)**5 + 5*P1*s*(s - 1)**4 - 5*P4*s**4*(s - 1) - 10*P2*s**2*(s - 1)**3 + 10*P3*s**3*(s - 1)**2
		Bdots = 5*P5*s**4 - 5*P4*s**4 - 5*P0*(s - 1)**4 + 5*P1*(s - 1)**4 + 20*P1*s*(s - 1)**3 - 20*P2*s*(s - 1)**3 - 20*P4*s**3*(s - 1) + 10*P3*s**3*(2*s - 2) - 30*P2*s**2*(s - 1)**2 + 30*P3*s**2*(s - 1)**2
		# Bddots = 20*P3*s**3 - 40*P4*s**3 + 20*P5*s**3 - 20*P0*(s - 1)**3 + 40*P1*(s - 1)**3 - 20*P2*(s - 1)**3 + 60*P1*s*(s - 1)**2 - 120*P2*s*(s - 1)**2 + 60*P3*s*(s - 1)**2 - 60*P4*s**2*(s - 1) - 30*P2*s**2*(2*s - 2) + 60*P3*s**2*(2*s - 2)

		Joint1_Angle = Q0[0] + (Qd[0] - Q0[0]) * B
		Joint2_Angle = Q0[1] + (Qd[1] - Q0[1]) * B
		Joint3_Angle = Q0[2] + (Qd[2] - Q0[2]) * B

		Joint1_Velocity = (Qd[0] - Q0[0]) * Bdots / T[-1]
		Joint2_Velocity = (Qd[1] - Q0[1]) * Bdots / T[-1]
		Joint3_Velocity = (Qd[2] - Q0[2]) * Bdots / T[-1]
		return Joint1_Angle.tolist(), Joint2_Angle.tolist(), Joint3_Angle.tolist(), Joint1_Velocity.tolist(), Joint2_Velocity.tolist(), Joint3_Velocity.tolist()
	
	
	def cmd_compose(self):
		if self.flag == 1:  # left leg
			self.cmd_array = [
				-self.Joint1_cmd.Position + leg1_bias[0], -self.Joint1_cmd.Velocity, -self.Joint1_cmd.Torque, self.Joint1_cmd.Kp, self.Joint1_cmd.Kd,
				 self.Joint2_cmd.Position + leg1_bias[1], self.Joint2_cmd.Velocity, self.Joint2_cmd.Torque, self.Joint2_cmd.Kp, self.Joint2_cmd.Kd,
				 self.Joint3_cmd.Position + leg1_bias[2], self.Joint3_cmd.Velocity, self.Joint3_cmd.Torque, self.Joint3_cmd.Kp, self.Joint3_cmd.Kd
			]

			print(self.cmd_array)
			
			'''safety check for command if winthin the range before send to publisher'''
			# if (	self.cmd_array[0] < Leg1_Joint1_limt[0] or self.cmd_array[0] > Leg1_Joint1_limt[1]
			# 	or 	self.cmd_array[5] < Leg1_Joint2_limt[0] or self.cmd_array[5] > Leg1_Joint2_limt[1]
			# 	or 	self.cmd_array[10]< Leg1_Joint3_limt[0] or self.cmd_array[10] > Leg1_Joint3_limt[1]):
			# 	self.Safety_Flag = Danger
			# 	self.Danger_info = Cmd_violate
			# 	# print(Leg1_Joint2_limt[0]-self.Joint2_cmd.Position)
			# 	# print(Leg1_Joint2_limt[1]-self.Joint2_cmd.Position)
			# 	# print(Leg1_Joint3_limt[0]-self.Joint3_cmd.Position)
			# 	# print(Leg1_Joint3_limt[1]-self.Joint3_cmd.Position)
			# 	self.safety_and_reset()

			# if (abs(self.Joint1_cmd.Position-self.Joint1_fb.Position)>PI/6.0 or abs(self.Joint3_cmd.Position-self.Joint3_fb.Position)>PI/6.0):
			# 	# or abs(self.Joint2_cmd.Position-self.Joint2_fb.Position)>PI/6.0 
			# 	self.Danger_info = Large_Error
			# 	self.Safety_Flag = Danger
			# 	self.safety_and_reset()
			

		elif self.flag == 2:  # right leg
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
				# self.Safety_Flag = Danger
				# self.Danger_info = Cmd_violate
				# # print(Leg1_Joint2_limt[0]-self.Joint2_cmd.Position)
				# # print(Leg1_Joint2_limt[1]-self.Joint2_cmd.Position)
				# # print(Leg1_Joint3_limt[0]-self.Joint3_cmd.Position)
				# # print(Leg1_Joint3_limt[1]-self.Joint3_cmd.Position)
				# self.safety_and_reset()
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
			Leg2_pub.publish(Leg_pub_cmd)				

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
	Leg1_pub = rospy.Publisher('/leg/leg2/cmd', Float32MultiArray, queue_size = 2)
	Leg2_pub = rospy.Publisher('/leg/leg2/cmd', Float32MultiArray, queue_size = 2)


	'''!!!!!!!!! Danger---Danger---Danger---Danger---Danger---Danger!!!!!!!!!
	Define leg after the pub function immediately and will be used immediately below in sub function
	!!!!!!!!!!!!!!!!!!!!!Do not Move to otehr place !!!!!!!!!!!!!!!!!!!!!!!!!
	!!!!!!!!! Danger---Danger---Danger---Danger---Danger---Danger!!!!!!!!!'''

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

	'''!!!!!!!!! Danger---Danger---Danger---Danger---Danger---Danger!!!!!!!!!
	Define Leg1 and Leg2 before Subscriber
	!!!!!!!!!!!!!!!!!!!!!Do not Move to otehr place !!!!!!!!!!!!!!!!!!!!!!!!!
	!!!!!!!!! Danger---Danger---Danger---Danger---Danger---Danger!!!!!!!!!'''

	# feedback
	#rospy.Subscriber('/leg/leg1/feedback', Float32MultiArray, Leg1_Cb)
	rospy.Subscriber('/leg/leg2/feedback', Float32MultiArray, Leg2_Cb)
	
	Frequency = 500
	dt = 1.0/Frequency
	rate = rospy.Rate(Frequency)
	
	'''verify that the robot position is updated for trajectory planing as the inital position'''

	'''plan a trajectory with time duration delta_T'''
	delta_T = 5.0
	T = np.linspace(0, delta_T, round(delta_T*Frequency+1), endpoint=True)
	Joint_Angle_C = [Leg1.Joint1_fb.Position, Leg1.Joint2_fb.Position+0.4, Leg1.Joint3_fb.Position]
	Q1, Q2, Q3, d_Q1, d_Q2, d_Q3 = Leg1.Traj_Plan(Joint_Angle_C, T)

	# New Trajectory
	
	#Q1 = Leg1.Joint1_fb.Position
	# Q2 = np.linspace(0, PI/2, round(delta_T*Frequency+1), endpoint=True)
	# d_Q2 = np.ones(1,round(PI/2*Frequency+1))


	'''print data to file'''
	File_name = time.strftime('%Y_%m_%d_%H_%M_%S')
	file = open('./Data/Exp_' + File_name + '.txt', 'w')
	time.sleep (1.5)
	
	

	i = 0
	count = 0
	loop_count = 0
	print('----------Rospy Loop Started----------')
	T0 = rospy.get_time ()
	while not rospy.is_shutdown():
		t = rospy.get_time() - T0
		yaw = 0
		
		'''publish the planned trajectory one point by one point'''
		
		if True:
			if count < 500*5:
				Leg2.Joint2_cmd.Position = 0.2
				Leg2.Joint3_cmd.Position = -math.pi/2
			
			elif (count>=500*5 and count<= 1000*5):
				Leg2.Joint2_cmd.Position += 0.2/500
				tmp1 = Leg2.Joint2_cmd.Position
				Leg2.Joint2_cmd.Velocity = 0
				Leg2.Joint3_cmd.Position -= 0
				tmp2 = Leg2.Joint3_cmd.Position
				
			elif (count>=1000*5 and count<= 1500*5):
				Leg2.Joint2_cmd.Position -= 0.2/500
				Leg2.Joint2_cmd.Velocity = 0.0
				Leg2.Joint3_cmd.Position += 0

			elif (count>1500*5):
				Leg2.Joint2_cmd.Position += 0
				Leg2.Joint3_cmd.Position += 0


			Leg1.Joint2_cmd.Velocity = 0
			Leg1.Joint1_cmd.Torque = 0.0# 0
			Leg1.Joint1_cmd.Kp = 100.0# 110.0
			Leg1.Joint1_cmd.Kd = 15.0# 6.0
			Leg1.Joint2_cmd.Torque = 0.0# 0
			Leg1.Joint2_cmd.Kp = 100.0# 110.0
			Leg1.Joint2_cmd.Kd = 15.0# 6.0
			Leg1.Joint3_cmd.Torque = 0.0# 0
			Leg1.Joint3_cmd.Kp = 110.0# 110.0
			Leg1.Joint3_cmd.Kd = 15.0# 6.0

			Leg2.Joint2_cmd.Velocity = 0
			Leg2.Joint1_cmd.Torque = 0.0# 0
			Leg2.Joint1_cmd.Kp = 0.0# 110.0
			Leg2.Joint1_cmd.Kd = 30.0# 6.0
			Leg2.Joint2_cmd.Torque = 0.0# 0
			Leg2.Joint2_cmd.Kp = 110.0# 110.0
			Leg2.Joint2_cmd.Kd = 6.0# 6.0
			Leg2.Joint3_cmd.Kp = 0.0# 110.0
			Leg2.Joint3_cmd.Kd = 0.0# 6.0

			count = count + 1 
			#print('real 2:',Leg2.Joint2_cmd.Position + leg2_bias[1])
			#print('real 3',Leg2.Joint3_cmd.Position + leg2_bias[2])
			
			i = i+1
			# print(i)
			# print(count)
			# print(i)
			# '''publish the planned trajectory one point by one point'''
			# '''check encoder reading and command sending are within the joint angle range
			# otherwise stop the leg'''
				# print(11111, Leg1.Joint1_fb.Position, Leg1.Joint2_fb.Position, Leg1.Joint3_fb.Position)
				# print(22222, Leg1.Joint1_cmd.Position, Leg1.Joint2_cmd.Velocity, Leg1.Joint3_cmd.Position)


		
		

		'''check encoder reading and command sending are within the joint angle range
		otherwise stop the leg'''
		# print(11111, Leg1.Joint1_fb.Position, Leg1.Joint2_fb.Position, Leg1.Joint3_fb.Position)
		# print(22222, Leg1.Joint1_cmd.Position, Leg1.Joint2_cmd.Velocity, Leg1.Joint3_cmd.Position)
		
		# if Leg1.Safety_Flag == Danger: 
		# 	Leg1.safety_and_reset()
		# 	rospy.signal_shutdown("Stopped! Error Occured!")
		# 	print("----------Rospy Loop Stopped!----------")
		# 	print("Leg_cmd: ", Leg1.cmd_array[0],  Leg1.cmd_array[5],  Leg1.cmd_array[10])
		# 	break
		# elif Leg1.Safety_Flag == Safe:
		# 	a = 1
		# 	Leg1.control()
			
		if Leg2.Safety_Flag == Danger: 
			Leg2.safety_and_reset()
			rospy.signal_shutdown("Stopped! Error Occured!")
			print("----------Rospy Loop Stopped!----------")
			print("Leg_cmd: ", Leg2.cmd_array[0],  Leg2.cmd_array[5],  Leg2.cmd_array[10])
			break
		elif Leg2.Safety_Flag == Safe:
			a = 1
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
