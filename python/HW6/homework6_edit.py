#!/usr/bin/env python

import sys
sys.path.append(r"/Users/robertbrothers/Desktop/Fall 2014/Fundamentals_of_Robotics/robo_git/python/")
import robotics_functions as rf
import matplotlib.pyplot as plt
import numpy as np
import sympy as sy
from mpl_toolkits.mplot3d import Axes3D


#########################		Problem 1 #######################################
# set up the D-H table
dh_table = np.array( [
	[0,-np.pi/2, 0, np.pi/4],
	[0, np.pi/2,.5, np.pi/4],
	[0, 0, .1, 0],
	[0,-np.pi/2, 0, np.pi/4],
	[0, np.pi/2, 0, np.pi/4],
	[0, 0, .5, np.pi/4]
	])

A0n = []
A06 = np.identity(4)
for link in dh_table:
	A06 = A06*rf.denavit_hartenberg(link)
	A0n.append(A06)
P = []
for i in A0n:
	P.append(i[:,3][:3])

X,Y,Z = [np.array([ p[i] for p in P]).T[0][0] for i in range(3)]

# Plot the Graph
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
line1 = ax.plot([X[0],X[1]], [Y[0], Y[1]], [Z[0], Z[1]], 'r')
line2 = ax.plot([X[1],X[2]], [Y[1], Y[2]], [Z[1], Z[2]], 'r')
line3 = ax.plot([X[2],X[3]], [Y[2], Y[3]], [Z[2], Z[3]], 'r')
line4 = ax.plot([X[3],X[4]], [Y[3], Y[4]], [Z[3], Z[4]], 'r')
line5 = ax.plot([X[4],X[5]], [Y[4], Y[5]], [Z[4], Z[5]], 'r')
plt.show()
######################		End of Problem1 ####################################

#######################		Problem 2 #########################################
[th1, th2, th3] = sy.symbols("th1 th2 th3")
dh_table = np.array([
	[0.5, 0, 0, -th1],
	[0.5, 0, 0, th2],
	[0.25,0, 0, th3]
	])
A0n
A03 = np.identity(4)
for link in dh_table:
	A03 = A03*rf.sym_denavit_hartenberg(link)
	A0n.append(A06)
ee_position = np.array([[.75],[0],[0]])
