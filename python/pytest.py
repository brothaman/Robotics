#!/usr/bin/env python

import robotics_functions as rf, numpy as np, scipy as sp, sympy as sy

[l1, l2, l3, t1, t2, t3, a1, a2, a3, d1, d2, d3] = sy.symbols("l1 l2 l3 t1 t2 t3 a1 a2 a3 d1 d2 d3")
[q1, q2, qdot1, qdot2, qddot1, qddot2, m1, m2, r1, r2] = sy.symbols("q1 q2 qdot1 qdot2 qddot1 qddot2 m1 m2 r1 r2")

link_list_cm = [[
		[l1, 0, 0, q1],
		[a2, 0, 0, q2]
		],
		[ sy.Matrix([[-l1/2],[0],[0],[1]]),
			sy.Matrix([[-a2/2],[0],[0],[1]])
			]
		]

m = np.array([m1, m2])
l = np.array([l1, l2])
r = np.array([r1, r2])


M = [sy.Matrix([
	[m[i],0,0],
	[0,m[i],0],
	[0,0,m[i]]
	]) for i in range(len(m))]
I = [sy.Matrix([
	[m1*l[0]**2/3,0,0],
	[0,m1*np.pi*r[0]**2/3, 0],
	[0, 0, m1*l[0]**2/3]
	]), 
	sy.Matrix([
		[m2*l[1]**2/3,0,0],
		[0,m2*l[1]**2/3,0],
		[0,0,m2*np.pi*r[1]**2/3]
		])
	]
q = sy.Matrix([
	[q1],
	[q2]
	])
qdot = sy.Matrix([
	[qdot1],
	[qdot2]
	])
tdv_vec = [
		(qdot1,qddot1),
		(qdot2,qddot2),
		(q1, qdot1),
		(q2, qdot2),
		]
if __name__ == "__main__":
	print sy.printing.latex(sy.simplify(sy.trigsimp(rf.sym_torque(link_list_cm, M, I, qdot, q, tdv_vec))))
