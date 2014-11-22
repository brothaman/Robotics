#!/usr/bin/env python

import robotics_functions as rf, numpy as np, scipy as sp, sympy as sy
[l1, l2, l3, t1, t2, t3, a1, a2, a3, d1, d2, d3] = sy.symbols("l1 l2 l3 t1 t2 t3 a1 a2 a3 d1 d2 d3")
[q1, q2, qdot1, qdot2, qddot1, qddot2, m1, m2, r1, r2, I1, I2] = sy.symbols("q1 q2 qdot1 qdot2 qddot1 qddot2 m1 m2 r1 r2 I1 I2")

link_list_cm = [[
		[ l1, 0, 0, q1],
		[ l2, 0, 0, q2]
		],
		[ sy.Matrix([[-l1/2],[0],[0],[1]]),
			sy.Matrix([[-l2/2],[0],[0],[1]])
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
	[I1,0,0],
	[0,I1, 0],
	[0, 0, I1]
	]), 
	sy.Matrix([
		[I2,0,0],
		[0,I2,0],
		[0,0,I2]
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
qddot = sy.Matrix([
	[qddot1],
	[qddot2],
	])
tdv_vec = [
		(qdot1,qddot1),
		(qdot2,qddot2),
		(q1, qdot1),
		(q2, qdot2),
		]
'''
time = np.array([0,1,3,5])
order = 3
rf.mat_builder( order, time)
'''

L = rf.sym_lagrangian(link_list_cm, M, I, qdot)[0]
print sy.printing.latex(sy.simplify(sy.trigsimp(L)))
eom = rf.equations_of_motion(L, q,qdot, qddot,tdv_vec)

eom1 = eom[0]
eom2 = eom[1]

G1 = sy.simplify(sy.trigsimp(eom1.subs(zip([qdot1,qdot2,qddot1,qddot2],[0,0,0,0]))))
G2 = sy.simplify(sy.trigsimp(eom2.subs(zip([qdot1,qdot2,qddot1,qddot2],[0,0,0,0]))))

C1 = sy.simplify(sy.trigsimp(eom1.subs(zip([qddot1,qddot2],[0,0]))-G1))
C2 = sy.simplify(sy.trigsimp(eom2.subs(zip([qddot1,qddot2],[0,0]))-G2))

M1 = sy.simplify(sy.trigsimp(eom1-C1-G1))
M2 = sy.simplify(sy.trigsimp((eom2-C2-G2)))


M11 = sy.simplify(sy.trigsimp(M1.subs(zip([qddot1,qddot2],[1,0]))))
M12 = sy.simplify(sy.trigsimp(M1.subs(zip([qddot1,qddot2],[0,1]))))

M21 = sy.simplify(sy.trigsimp(M2.subs(zip([qddot1,qddot2],[1,0]))))
M22 = sy.simplify(sy.trigsimp(M2.subs(zip([qddot1,qddot2],[0,1]))))

print("G1 = " + str(G1))
print("G2 = " + str(G2))
print("C1 = " + str(C1))
print("C2 = " + str(C2))
print("M11 = " + str(M11))
print("M12 = " + str(M12))
print("M21 = " + str(M21))
print("M22 = " + str(M22))
if __name__ == "__main__":
	'''
	sy.pprint(sy.simplify(sy.trigsimp(eom1)))
	sy.pprint(sy.simplify(sy.trigsimp(eom2)))
	'''
