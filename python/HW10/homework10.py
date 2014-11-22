#!/usr/bin/env python

import sys
sys.path.append(r"/Users/robertbrothers/Desktop/Fall 2014/Fundamentals_of_Robotics/robo_git/python/")
import robotics_functions as rf, sympy as sy, numpy as np

[l1,l2,l3,m1,m2,m3,n1,n2,n3] = sy.symbols("l1,l2,l3,m1,m2,m3,n1,n2,n3")
[theta1,theta2,theta3] = sy.symbols("theta1 theta2 theta3")
[thetadot1,thetadot2,thetadot3] = sy.symbols("thetadot1,thetadot2,thetadot3")
[thetaddot1,thetaddot2,thetaddot3] = sy.symbols("thetaddot1,thetaddot2,thetaddot3")
[Ixx1,Ixx2,Ixx3,Iyy1,Iyy2,Iyy3,Izz1,Izz2,Izz3] = sy.symbols("Ixx1,Ixx2,Ixx3,Iyy1,Iyy2,Iyy3,Izz1,Izz2,Izz3")
Mmat = [m1,m2,m3]

I = [sy.Matrix([
	[Ixx1,0,0],
	[0,Iyy1,0],
	[0,0,Izz1]]),
	sy.Matrix([
	[Ixx2,0,0],
	[0,Iyy2,0],
	[0,0,Izz2]]),
	sy.Matrix([
	[Ixx3,0,0],
	[0,Iyy3,0],
	[0,0,Izz3]])
]
M = [sy.Matrix([
	[val,0,0],
	[0,val,0],
	[0,0,val]]) for val in Mmat]
link_list = [
		[l1,0,0,theta1],
		[l2,0,0,theta2],
		[l3,0,0,theta3],
		]

cm = [
		sy.Matrix([-n1,0,0,1]),
		sy.Matrix([-n2,0,0,1]),
		sy.Matrix([-n3,0,0,1])
		]

link_list_cm = [link_list, cm]

q = sy.Matrix([
	[theta1],
	[theta2],
	[theta3],
	])

qdot = sy.Matrix([
	[thetadot1],
	[thetadot2],
	[thetadot3],
	])

qddot = sy.Matrix([
	[thetaddot1],
	[thetaddot2],
	[thetaddot3],
	])

tdv = [
		(thetadot1, thetaddot1),
		(thetadot2, thetaddot2),
		(thetadot3, thetaddot3),
		(theta1, thetadot1),
		(theta2, thetadot2),
		(theta3, thetadot3)
		]

J = rf.sym_pt_jacobian(link_list_cm)
L = rf.sym_lagrangian( link_list_cm, M, I, qdot)
[t1, t2, t3] = rf.equations_of_motion( L[0], q, qdot, tdv)
print "Jacobian"
for j in J:
	print(sy.simplify(sy.trigsimp(j)))
print "\nTheta 1\n"
print(sy.simplify(sy.trigsimp(t1)))
print "\nTheta 2\n"
print(sy.simplify(sy.trigsimp(t2)))
print "\nTheta 3\n"
print(sy.simplify(sy.trigsimp(t3)))
print "\nThe Lagrangian\n"
print(sy.simplify(sy.trigsimp(L)[0]))
