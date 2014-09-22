import numpy as np

def my_rotation( theta=0):
	theta = theta*np.pi/180.0
	return np.matrix([[ np.cos(theta), np.sin(theta), 0], [ -np.sin(theta), np.cos(theta), 0],[ 0, 0, 1]])
