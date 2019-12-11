#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm
from lab6_header import *
"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def skewS(S):
	w = S[0:3]
	v = S[3:6]
	
	SS = np.zeros((4,4))

	SS[0,0] = 0
	SS[0,1] = -w[2]
	SS[0,2] = w[1]
	SS[0,3] = v[0]
	SS[1,0] = w[2]
	SS[1,1] = 0
	SS[1,2] = -w[0]
	SS[1,3] = v[1]
	SS[2,0] = -w[1]
	SS[2,1] = w[0]
	SS[2,2] = 0
	SS[2,3] = v[2]
	SS[3,0] = 0
	SS[3,1] = 0
	SS[3,2] = 0
	SS[3,3] = 0

	SS = np.asarray(SS, dtype=np.float64)

	return SS


def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))

	M = [[0, -1,  0, 0.54], 
		 [0,  0, -1, .254],
		 [1,  0,  0, .206], 
		 [0,  0,  0,    1]]

	S = np.array([[0,      0,      0,      0,     1,      0],
				  [0,      1,      1,      1,     0,      1],
				  [1,      0,      0,      0,     0,      0],
				  [0, -0.152, -0.152, -0.152,     0, -0.152],
				  [0,      0,      0,      0, 0.152,      0],
				  [0,      0,  0.244,  0.457, -0.11,   0.54]])
	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	#print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()

	T = expm(skewS(S[:,0])*theta1)
	T = np.matmul(T, expm(skewS(S[:,1])*theta2))
	T = np.matmul(T, expm(skewS(S[:,2])*theta3))
	T = np.matmul(T, expm(skewS(S[:,3])*theta4))
	T = np.matmul(T, expm(skewS(S[:,4])*theta5))
	T = np.matmul(T, expm(skewS(S[:,5])*theta6))
	T = np.matmul(T,M)

	# ==============================================================#
	
	#print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):

    # theta1 to theta6
	thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	l01 = 0.152
	l02 = 0.120
	l03 = 0.244
	l04 = 0.093
	l05 = 0.213
	l06 = 0.083
	l07 = 0.083
	l08 = 0.082    
	l09 = 0.0535
	l10 = 0.059   # thickness of aluminum plate is around 0.006

	xgrip = float(xWgrip) + 0.1490
	ygrip = float(yWgrip) - 0.1490
	zgrip = float(zWgrip) - 0.0087 # Offset to foam board  #0.0193 offset to benchtop
	yaw = np.radians(yaw_WgripDegree)

	xcen = xgrip - l09*np.cos(yaw)
	ycen = ygrip - l09*np.sin(yaw)
	zcen = zgrip

	L = l06 + l02 - l04

	thetas[0] = np.arctan2(ycen, xcen) - np.arcsin(L / np.sqrt(xcen**2 + ycen**2))

	thetas[5] = PI/2 + thetas[0] - yaw
	
	x3end = xcen - l07*np.cos(thetas[0]) + L*np.sin(thetas[0])
	y3end = ycen - l07*np.sin(thetas[0]) - L*np.cos(thetas[0])
	z3end = zcen + l10 + l08

	d4 = np.sqrt(x3end**2 + y3end**2 + (z3end - l01)**2)

	b1 = np.arcsin((z3end - l01) / d4)

	b2 = np.arccos((l05**2 - l03**2 - d4**2) / (-2*l03*d4))

	b3 = np.arccos((d4**2 - l03**2 - l05**2) / (-2*l03*l05))

	thetas[1] = -(b1 + b2)

	thetas[2] = PI - b3
	
	thetas[3] = -(thetas[2] + thetas[1])

	thetas[4]= -PI / 2

	#print("theta1 to theta6: " + str(thetas) + "\n")

	return lab_fk(float(thetas[0]), float(thetas[1]), float(thetas[2]), \
		          float(thetas[3]), float(thetas[4]), float(thetas[5]) )


