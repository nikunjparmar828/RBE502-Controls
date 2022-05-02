#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin, acos, asin
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os

class Quadrotor():
	#xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot = [0,0,0,0,0,0,0,0,0]

	def __init__(self):
		# publisher for rotor speeds
		self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)
		
		# subscribe to Odometry topic

		self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry",Odometry, self.odom_callback)
		self.t0 = None
		self.t = None
		self.t_series = []
		self.x_series = []
		self.y_series = []
		self.z_series = []
		self.mutex_lock_on = False
		rospy.on_shutdown(self.save_data)

		self.w1 = 0
		self.w2 = 0
		self.w3 = 0
		self.w4 = 0
		self.rho = 0

		# TODO: include initialization codes if needed
		self.m = 0.027 # mass in kg
		self.l = 0.046 # quadrotor length in meter 

		#inertia terms 
		self.Ix = 16.571710 * (10**(-6))
		self.Iy = 16.571710 * (10**(-6))
		self.Iz = 29.261652 * (10**(-6))
		self.Ip = 12.65625 * (10**(-8))

		# propeller thrust and moment factors 
		self.kf = 1.28192 * (10**(-8))
		self.km = 5.964552 * (10**(-3))

		# TUNE
		self.Kp = 10 # using kp and kd in 'traj_evaluate' function to calculate desired theta and phi values 
		self.Kd = 2

		self.lambda1 = 100
		self.lambda2 = 100
		self.lambda3 = 100
		self.lambda4 = 100

		self.g = 9.82 

		self.k1 = 0.5
		self.k2 = 1
		self.k3 = 1
		self.k4 = 1

		# self.time_list = [5,20,35,50,65]
		self.time_list = [5]
		self.t_count = 0

		self.s1 = 0
		self.s2 = 0
		self.s3 = 0
		self.s4 = 0


	def traj_evaluate(self):
		# TODO: evaluating the corresponding trajectories designed in Part 1to return the desired positions, velocities and accelerations
		# print(self.t)

		xd = (-1/42525000)* ((self.t)**5) + (7/1215000) * ((self.t)**4) + (-251/567000)*((self.t)**3) + (113/9720)*((self.t)**2) + (-325/6804)*(self.t)
		xd_dot = 5*(-1/42525000)* ((self.t)**4) + 4*(7/1215000) * ((self.t)**3) + 3*(-251/567000)*((self.t)**2) + 2*(113/9720)*((self.t)) + (-325/6804)
		xd_ddot = 20*(-1/42525000)* ((self.t)**3) + 12*(7/1215000) * ((self.t)**2) + 6*(-251/567000)*(self.t) + 2*(113/9720) 


		yd = (2/26578125)* ((self.t)**5) + (-7/607500) * ((self.t)**4) + (79/141750)*((self.t)**3) + (-211/24300)*((self.t)**2) + (1313/42525)*(self.t)
		yd_dot = 5*(2/26578125)* ((self.t)**4) + 4*(-7/607500) * ((self.t)**3) + 3*(79/141750)*((self.t)**2) + 2*(-211/24300)*(self.t) + (1313/42525)
		yd_ddot = 20*(2/26578125)* ((self.t)**3) + 12*(-7/607500) * ((self.t)**2) + 6*(79/141750)*(self.t) + 2*(-211/24300)

		zd = (1/11375000)* ((self.t)**5) + (-1/65000) * ((self.t)**4) + (89/91000)*((self.t)**3) + (-71/2600)*((self.t)**2) + (2857/9100)*(self.t)
		zd_dot = 5*(1/11375000)* ((self.t)**4) + 4*(-1/65000) * ((self.t)**3) + 3*(89/91000)*((self.t)**2) + 2*(-71/2600)*(self.t) + (2857/9100)
		zd_ddot = 20*(1/11375000)* ((self.t)**3) + 12*(-1/65000) * ((self.t)**2) + 6*(89/91000)*(self.t) + 2*(-71/2600)

		# # Fx = self.m *(-self.Kp * (x-xd) - self.Kd * (x_dot - xd_dot) + xd_ddot)
		# # Fy = self.m *(-self.Kp * (y-yd) - self.Kd * (y_dot - yd_dot) + yd_ddot)

		# xd = (-1/42525000)* ((t)**5) + (7/1215000) * ((t)**4) + (-251/567000)*((t)**3) + (113/9720)*((t)**2) + (-325/6804)*(t)
		# xd_dot = 5*(-1/42525000)* ((t)**4) + 4*(7/1215000) * ((t)**3) + 3*(-251/567000)*((t)**2) + 2*(113/9720)*((t)) + (-325/6804)
		# xd_ddot = 20*(-1/42525000)* ((t)**3) + 12*(7/1215000) * ((t)**2) + 6*(-251/567000)*(t) + 2*(113/9720) 


		# yd = (2/26578125)* ((t)**5) + (-7/607500) * ((t)**4) + (79/141750)*((t)**3) + (-211/24300)*((t)**2) + (1313/42525)*(t)
		# yd_dot = 5*(2/26578125)* ((t)**4) + 4*(-7/607500) * ((t)**3) + 3*(79/141750)*((t)**2) + 2*(-211/24300)*(t) + (1313/42525)
		# yd_ddot = 20*(2/26578125)* ((t)**3) + 12*(-7/607500) * ((t)**2) + 6*(79/141750)*(t) + 2*(-211/24300)

		# zd = (1/11375000)* ((t)**5) + (-1/65000) * ((t)**4) + (89/91000)*((t)**3) + (-71/2600)*((t)**2) + (2857/9100)*(t)
		# zd_dot = 5*(1/11375000)* ((t)**4) + 4*(-1/65000) * ((t)**3) + 3*(89/91000)*((t)**2) + 2*(-71/2600)*(t) + (2857/9100)
		# zd_ddot = 20*(1/11375000)* ((t)**3) + 12*(-1/65000) * ((t)**2) + 6*(89/91000)*(t) + 2*(-71/2600)
 


		return (xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot)

	def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
		# obtain the desired values by evaluating the correspondingtrajectories

		# state space representation of the system 

		x1 = xyz[2,0] # z 

		# print(x1)

		x2 = rpy[0,0] # roll
		x3 = rpy[1,0] # pitch
		x4 = rpy[2,0] # yaw

		# print(x2,x3,x4);

		x5 = xyz_dot[2,0] # z_dot
		x6 = rpy_dot[0,0] # roll velocity
		x7 = rpy_dot[1,0] # pitch velocity 
		x8 = rpy_dot[2,0] # yaw velocity

		
		xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot = self.traj_evaluate()

		# xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot = self.traj_evaluate(self.time_list[0])
		# if self.t_count == 0:
		# 	xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot = self.traj_evaluate(self.time_list[0])
		# 	self.t_count+=1

		# elif self.s1 < 0.00000015 and self.s2 < 0.00000015 and self.s3 < 0.00000015 and self.s4 < 0.00000015:
		# 	xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot = self.traj_evaluate(self.time_list[self.t_count])
		# 	self.t_count+=1
		# else:
		# 	xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot = self.traj_evaluate(self.time_list[self.t_count-1])


		# TODO: implement the Sliding Mode Control laws designed in Part 2 to calculate the control inputs "u"

		## first control input -------------------------------------------- 

		self.s1 = (x5-zd_dot) + self.lambda1*(x1-zd)

		#u1 = -(((self.m*(self.lambda1*(x5-zd_dot)-zd_ddot) - self.g)/(cos(x2)*cos(x3))) + self.k1)*self.sat(self.s1)
		# u1 = self.m/(cos(x1)*cos(x2))*(self.g + zd_ddot - self.lambda1*(x5 - zd_dot) - self.k1*self.sat(self.s1))
		u1 = self.m*(self.g + zd_ddot - self.lambda1*(x5 - zd_dot) - self.k1*self.sat(self.s1))/(cos(x2)*cos(x3))

		# print("U1: " + str(u1))
		print("z: " + str(x1))
		print("zd: " + str(zd))
		# print("xd: " + str(xd))
		# print("yd: " + str(yd))
		
		# -----------------------------------------------------------------

		# After calculating u1 
		# we need to figure out desired 'theta' and 'phi'

		Fx = self.m *(-self.Kp * (xyz[0,0]-xd) - self.Kd * (xyz_dot[0,0] - xd_dot) + xd_ddot)
		Fy = self.m *(-self.Kp * (xyz[1,0]-yd) - self.Kd * (xyz_dot[1,0] - yd_dot) + yd_ddot)

		# print("Fx",Fx)
		# print("Fy",Fy)

		if u1 == 0:
			print("------------------------------------------------------------------------------------------")

		# theta_d = self.wrap_pipi(asin(Fx/u1)) # theta desired 
		# PHI_d = self.wrap_pipi(asin(-Fy/u1)) # phi desires --> not using the variable name 'phi' because I chose it as a tuning parameter name
		# psi_d = 0

		theta_d = 0 # theta desired 
		PHI_d = 0 # phi desires --> not using the variable name 'phi' because I chose it as a tuning parameter name
		psi_d = 0
		# print("theta_d", theta_d)
		# print("PHI_d", PHI_d)
		# theta_d_dot = 0
		# PHI_d_dot = 0
		# psi_d_dot = 0

		# theta_d_ddot = 0
		# PHI_d_ddot = 0
		# psi_d_ddot = 0

		## second control input -------------------------------------------- 

		self.s2 = self.wrap_pipi(rpy_dot[0,0]) + self.lambda2*self.wrap_pipi(rpy[0,0]-PHI_d)

		u2 = -(x7*x8*(self.Iy-self.Iz) - self.Ip*self.max_ohm()*x7 + self.lambda2*self.Ix*x6 + self.k2*self.Ix*self.sat(self.s2))
		# u2 = -x7*x8*(self.Iy - self.Iz) - self.lambda2*self.Ix*x6 - self.Ix*(self.rho + self.k2)*self.sat(self.s2) + self.Ip*(self.w1 - self.w2 + self.w3 - self.w4)*x7
		# u2 = 0


		# u2 = -x7*x8*(self.Iy - self.Iz) + self.Ip*self.max_ohm()*x7 - self.lambda2*(x6) - self.k2*self.sat(self.s2)


		# print("U2: " + str(u2))
		# print("phi: " + str(x2))
		# # print("Fy: " + str(Fy))
		# print("phi_d: " + str(PHI_d))
		## third control input -------------------------------------------- 
		
		self.s3 = self.wrap_pipi(rpy_dot[1,0]) + self.lambda3*self.wrap_pipi(rpy[1,0]-theta_d)

		u3 = -(x6*x8*(self.Iz-self.Ix) + self.Ip*self.max_ohm()*x6 + self.lambda3*self.Iy*x7 + self.k3*self.Iy*self.sat(self.s3))
		# u3 = -x6*x8*(self.Iz - self.Ix) - self.Ip*(self.w1 - self.w2 + self.w3 - self.w4) - self.lambda3*self.Iy*x7 - self.Iy*(self.rho + self.k3)*self.sat(self.s3)
		# u3 = 0

		# u3 = -x6*x8*(self.Iz - self.Ix) - self.Ip*self.max_ohm()*x6 - self.lambda3*(x7) - self.k3*self.sat(self.s3)

		#print("U3: " + str(u3))
		#print("theta: " + str(x3))

		## fourth control input -------------------------------------------- 

		self.s4 = self.wrap_pipi(rpy_dot[2,0]) + self.lambda4*self.wrap_pipi(rpy[2,0]-psi_d)

		u4 = -(x6*x7*(self.Ix - self.Iy) + self.lambda4 * self.Iz * x8 + self.k4*self.Iz*self.sat(self.s4))
		# u4 = -x6*x7*(self.Ix - self.Iy) - self.lambda4 * self.Iz * x8 - self.Iz*(self.rho - self.k4)*self.sat(self.s4)
		# u4 = 0

		# u4 = -x6*x7*(self.Ix - self.Iy) - self.lambda4 * x8 - self.k4*self.sat(self.s4) 
		#print("U4: " + str(u4))
		#print("psi: " + str(x4))

		# REMARK: wrap the roll-pitch-yaw angle errors to [-pi to pi]



		# TODO: convert the desired control inputs "u" to desired rotor velocities "motor_vel" by using the "allocation matrix"

		aa = (1/(4*self.kf))
		bb =  (1.4142/(4*self.kf*self.l))
		cc = (1/4*self.km*self.kf)

		# if (aa*u1 - bb*u2 - bb*u3 - cc*u4 )< 0:
		# 	self.w1 = 0
		# else :
		# self.w1 = sqrt(aaa*u1 - bb*u2 - bb*u3 - cc*u4)

		self.w1 = sqrt(abs(aa*u1 - bb*u2 - bb*u3 - cc*u4))

		if self.w1>2618:
			# print('overshoot --> w1')
			self.w1 = 2618


		# if (aa*u1 - bb*u2 + bb*u3 + cc*u4 )< 0:
		#  	self.w2 = 0
		# else :
		 # self.w2 = sqrt(aa*u1 - bb*u2 + bb*u3 + cc*u4 )
		
		
		self.w2 = sqrt(abs(aa*u1 - bb*u2 + bb*u3 + cc*u4))

		if self.w2>2618:
			# print('overshoot --> w2')
			self.w2 = 2618

		# if (aa*u1 + bb*u2 + bb*u3 - cc*u4 )< 0:
		#  	self.w3 = 0
		# else :
		# self.w3 = sqrt(aa*u1 + bb*u2 + bb*u3 - cc*u4 )
		

		self.w3 = sqrt( abs(aa*u1 + bb*u2 + bb*u3 - cc*u4) )

		if self.w3>2618:
			# print('overshoot --> w3')
			self.w3 = 2618


		# if (aa*u1 + bb*u2 - bb*u3 + cc*u4 )< 0:
		#  	self.w4 = 0
		# else :
		# self.w4 = sqrt(aa*u1 + bb*u2 - bb*u3 + cc*u4 )



		self.w4 = sqrt( abs(aa*u1 + bb*u2 - bb*u3 + cc*u4) )

		if self.w4>2618:
			# print('overshoot --> w4')
			self.w4 = 2618

		# TODO: maintain the rotor velocities within the valid range of [0 to 2618]

		motor_vel = np.asarray([[self.w1],[self.w2],[self.w4],[self.w4]])
		# motor_vel = [[w1],[w2],[w4],[w4]]
		# publish the motor velocities to the associated ROS topic

		motor_speed = Actuators()
		motor_speed.angular_velocities = [motor_vel[0,0], motor_vel[1,0],motor_vel[2,0], motor_vel[3,0]]

		self.motor_speed_pub.publish(motor_speed)

	def wrap_pipi(self, angle):

		if abs(angle) > pi:
			return (angle%(pi))
		else:
			return angle



	def max_ohm(self):
		
		# ohn term calculation 
		# max_ohm = max(w1+w3)-min(w2+w4)
		# max_ohm = 2*max(w) - 0
		# max_ohm = 2*max(w)

		# 2*2618
		return self.w1 + self.w3 - self.w2 - self.w4 

	def calc_ohm(self):
		return (self.w1 - self.w2 + self.w3 - self.w4)

	def sat(self,s):
		# TUNE
		phi = 0.2

		return min(max(s/phi,-1),1)

	# odometry callback function (DO NOT MODIFY)
	def odom_callback(self, msg):
		if self.t0 == None:
			self.t0 = msg.header.stamp.to_sec()
		self.t = msg.header.stamp.to_sec() - self.t0
		
		# convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
		w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
		
		v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
		
		xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])

		# print(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
		
		q = msg.pose.pose.orientation
		
		T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
		
		T[0:3, 3] = xyz[0:3, 0]
		
		R = T[0:3, 0:3]
		
		xyz_dot = np.dot(R, v_b)
		
		rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
		
		rpy_dot = np.dot(np.asarray([[1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],[0, np.cos(rpy[0]), -np.sin(rpy[0])],[0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]]), w_b)
		
		rpy = np.expand_dims(rpy, axis=1)
		

		# store the actual trajectory to be visualized later
		if (self.mutex_lock_on is not True):
			self.t_series.append(self.t)
			self.x_series.append(xyz[0, 0])
			self.y_series.append(xyz[1, 0])
			self.z_series.append(xyz[2, 0])
		
		# call the controller with the current states
		self.smc_control(xyz, xyz_dot, rpy, rpy_dot)



	# save the actual trajectory data
	def save_data(self):
		# TODO: update the path below with the correct path
		
		with open("/home/nikunj/rbe502_project/src/project/scripts/log.pkl","wb") as fp:
		
			self.mutex_lock_on = True
		
		pickle.dump([self.t_series,self.x_series,self.y_series,self.z_series], fp)

if __name__ == '__main__':
	rospy.init_node("quadrotor_control")
	rospy.loginfo("Press Ctrl + C to terminate")
	whatever = Quadrotor()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")
