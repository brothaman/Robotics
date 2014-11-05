#!/usr/bin/env python

import sys
sys.path.append(r"/Users/robertbrothers/Desktop/Fall 2014/Fundamentals_of_Robotics/robo_git/python/")
import robotics_functions as rf
import sympy as sy
import numpy as np


[a1,a2,a3,l1,l2,l3,d1,d2,d3,t1,t2,t3] = sy.symbols("a1,a2,a3,l1,l2,l3,d1,d2,d3,t1,t2,t3")
################## Problem 1 ############################
print "########### Problem 1 #############################"
link_list = [
		[ 0, np.pi/2, 0, t1],
		[ 0,-np.pi/2,d2,  0],
		[l3, 0, 0, t3]
		]

# a)
print "################## a) ####################################"
J_end = rf.symbolic_jacobian(link_list)
Je = J_end[0]
'''
for i in range(len(J_end)-1):
	j = i+1
	Je = sy.Matrix.hstack(Je, J_end[j])
J_end = Je
'''
sy.pprint(J_end)

# b)
print "########################## b) ################################"
speed = np.ones(len(link_list))
speed = np.hstack((speed, np.zeros(3)))
speed = sy.Matrix(speed).T
speed = J_end.T*speed
sy.pprint(speed)


##########################################################



##################### Problem 2 ###########################
print "####################### Problem 2 ########################"
link_list = [
		[ 0, 0, l1, t1],
		[l2, np.pi/2, 0, t2],
		[l3, 0, 0, t3]
		]

# a) On paper - no function written for this yet

# b) jacobian of the end effector
J_end = rf.symbolic_jacobian(link_list)
sy.pprint(J_end)



############################################################


##################### Problem 3 ############################
print "########### Problem 3 ##########"
link_list = [
		[a1, 0, 0, t1],
		[a2, 0, 0, t2]
		]

# a) On paper - no function written for this yet
# b) On paper - no function written for this yet
# c) On paper - no function written for this yet
