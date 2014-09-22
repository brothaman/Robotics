import numpy as np

def my_rotation( theta=0):
	theta = theta*np.pi/180.0
	return np.matrix([[ np.cos(theta), np.sin(theta), 0], [ -np.sin(theta), np.cos(theta), 0],[ 0, 0, 1]])

# My_rotation file rotation matrix of pi/2

import my_rotation_function as mrf

print mrf.my_rotation(90);

# My_rotations file rotation matrices of theta from 0 to pi

import my_rotation_function as mrf
import numpy as np
a = []

for i in range(101):
	a.append( mrf.my_rotation(np.pi*i/1000.0))

print a
