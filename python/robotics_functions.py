import numpy as np, sympy as sy

gravity = sy.symbols("g")

def symsum( listin):
	val = sy.zeros(listin[0].shape)
	for i in listin:
		val = val+i
	return val


#define numerical rotation matricies
def rotation_z(theta):
	return np.matrix([
		[ np.cos(theta), -np.sin(theta), 0],
		[ np.sin(theta),  np.cos(theta), 0],
		[0, 0, 1]
		])
def rotation_y(theta):
	return np.matrix([
		[ np.cos(theta), 0,  np.sin(theta)],
		[0, 1, 0],
		[-np.sin(theta), 0,  np.cos(theta)]
		])
def rotation_x(theta):
	return np.matrix([
		[1, 0, 0],
		[0, np.cos(theta), -np.sin(theta)],
		[0, np.sin(theta),  np.cos(theta)]
		])

# define symbolic rotation matricies
def sym_rotation_z(theta):
	return sy.Matrix([
		[ sy.cos(theta), -sy.sin(theta), 0],
		[ sy.sin(theta),  sy.cos(theta), 0],
		[0, 0, 1]
		])
def sym_rotation_y(theta):
	return sy.Matrix([
		[ sy.cos(theta), 0,  sy.sin(theta)],
		[0, 1, 0],
		[-sy.sin(theta), 0,  sy.cos(theta)]
		])
def sym_rotation_x(theta):
	return sy.Matrix([
		[1, 0, 0],
		[0, sy.cos(theta), -sy.sin(theta)],
		[0, sy.sin(theta),  sy.cos(theta)]
		])

# define numerical translation matricies
def translation_z( d):
	return np.matrix([
		[1,0,0,0],
		[0,1,0,0],
		[0,0,1,d],
		[0,0,0,1]
		])
def translation_y( d):
	return np.matrix([
		[1,0,0,0],
		[0,1,0,d],
		[0,0,1,0],
		[0,0,0,1]
		])
def translation_x( d):
	return np.matrix([
		[1,0,0,d],
		[0,1,0,0],
		[0,0,1,0],
		[0,0,0,1]
		])

# define symbolic translation matrices
def sym_translation_z( d):
	return sy.Matrix([
		[1,0,0,0],
		[0,1,0,0],
		[0,0,1,d],
		[0,0,0,1]
		])
def sym_translation_y( d):
	return sy.Matrix([
		[1,0,0,0],
		[0,1,0,d],
		[0,0,1,0],
		[0,0,0,1]
		])
def sym_translation_x( d):
	return sy.Matrix([
		[1,0,0,d],
		[0,1,0,0],
		[0,0,1,0],
		[0,0,0,1]
		])
# numerically convert 3x3 rotation to 4x4 rotation
def convert3x3to4x4( matrix):
	# add column of zeroes
	matrix = np.hstack((matrix, np.transpose([np.zeros(3)])))
	# add row of 0,0,0,1
	matrix = np.vstack((matrix, np.array([0,0,0,1])))
	return matrix
# symbolically convert 3x3 rotation to 4x4 rotation
def syms_convert3x3to4x4( matrix):
	# add column of zeroes
	matrix = sy.Matrix.hstack(matrix, sy.Matrix(sy.zeros(3)[:,2]))
	# add row of 0,0,0,1
	matrix = sy.Matrix.vstack((matrix, sy.Matrix([0,0,0,1]).T))
	return matrix

def denavit_hartenberg( link):
	return (
		translation_z(link[2])*convert3x3to4x4(rotation_z(link[3]))*
		translation_x(link[0])*convert3x3to4x4(rotation_x(link[1]))
		)
def sym_denavit_hartenberg( link):
	return (
		sym_translation_z(link[2])*sy.Matrix(syms_convert3x3to4x4(sym_rotation_z(link[3])))*
		sym_translation_x(link[0])*sy.Matrix(syms_convert3x3to4x4(sym_rotation_x(link[1])))
		)

def get_A0n( link_list):
	A0i = np.identity(4)
	A0n = []
	for link in link_list:
		A0i = A0i*denavit_hartenberg( link)
		A0n.append(A0i)
	return A0n

def sym_get_A0n( link_list):
	A0i = np.identity(4)
	A0n = []
	for link in link_list:
		A0i = A0i*sym_denavit_hartenberg( link)
		A0n.append(A0i)
	return A0n

def get_A0i():
	A0i = []
	for link in link_list:
		A0i.append(denavit_hartenberg( link))
	return A0i

def sym_get_A0i(link_list):
	A0i = []
	for link in link_list:
		A0i.append(sym_denavit_hartenberg( link))
	return A0i


def end_jacobian( link_list):
	A0n = get_A0n( link_list)
	# rotational vectors
	R = [np.matrix(np.identity(3))]
	R = R + [end[:3,:3] for end in A0n]
	# positions of each end effector
	O = [sy.Matrix([[0],[0],[0]])]
	O = O + [np.matrix(end[:,3][:3]) for end in A0n]
	# unit z vector
	k = np.matrix([[0],[0],[1]])
	J_v = []
	J_w = []
	for i in range(len(A0n)):
		# if theta_i is 0 then joint is prismatic
		if (link_list[i][3] == 0):
			J_v.append(sy.Matrix(R[i]*k))
			J_w.append(sy.Matrix([[0],[0],[0]]))
		# if theta_i is not 0 then joint is revolute
		else:
			J_v.append( np.matrix(np.cross( (R[i]*k).T, (O[-1]-O[i]).T)).T )
			J_w.append(np.matrix(R[i]*k))
	J = [np.vstack( (J_v[i], J_w[i])) for i in range(len(J_v))]
	J = np.hstack(J)
	return J


def sym_end_jacobian( link_list):
	A0n = sym_get_A0n( link_list)
	# rotational vectors
	R = [sy.Matrix(np.identity(3))]
	R = R + [end[:3,:3] for end in A0n]
	# positions of each end effector
	O = [sy.Matrix([[0],[0],[0]])]
	O = O + [sy.Matrix(end[:,3][:3]) for end in A0n]
	# unit z vector
	k = sy.Matrix([[0],[0],[1]])
	J_v = []
	J_w = []
	for i in range(len(A0n)):
		# if theta_i is 0 then joint is prismatic
		if (link_list[i][3] == 0):
			J_v.append(sy.Matrix(R[i]*k))
			J_w.append(sy.Matrix([[0],[0],[0]]))
		# if theta_i is not 0 then joint is revolute
		else:
			J_v.append((sy.Matrix(R[i]*k).cross(O[-1]-O[i])))
			J_w.append(sy.Matrix(R[i]*k))
	J = [sy.Matrix.vstack( J_v[i], J_w[i]) for i in range(len(J_v))]
	J = sy.Matrix.hstack(J)
	Je= J[0]
	for i in range(len(J)-1):
		j = i+1
		Je = sy.Matrix.hstack(Je, J[j])
	return Je

def sym_cm_jacobian(link_list):
	return 0#Jcm

def sym_pt_jacobian(link_list_position):
	k = sy.Matrix([[0],[0],[1]])
	[link_list, pt_list] = link_list_position
	A0i = sym_get_A0n(link_list)
	O0i = [A0i[i]*pt_list[i] if type(pt_list[i])== type(A0i[i]) else 
			sy.Matrix(A0i[i][:,3][:3]) for i in range(len(A0i)) ]
	A0i = [sy.eye(4)]+A0i
	Jpt = []
	for i in range(len(link_list)):
		j_pt = []
		jci = sy.Matrix(np.zeros(6)).T
		for j in range(len(link_list)):
			if j <= i:
				if not link_list[j][-1] == 0:
					j_pt.append( sy.Matrix.vstack((A0i[j][:3,:3]*k).cross(
						sy.Matrix(O0i[i][:3])-sy.Matrix(A0i[j][:,3][:3])),
						A0i[j][:3,:3]*k)
						)
				else:
					j_pt.append( sy.Matrix.vstack(A0i[j][:3,:3]*k, sy.Matrix([[0],[0],[0]])))
			else:
				j_pt.append( sy.Matrix(np.zeros(6)).T)
		q = j_pt[0]
		for i in range(len(j_pt)-1):
			j = i+1
			q = sy.Matrix.hstack(q, j_pt[j])
		Jpt.append(q)
	return Jpt



#def rotational_velocity_jacobian( link_list)
#def jacobian(link_list)
def D_i( vec):
	[Ji, Mi, Ii, Ri] = vec
	Jv = Ji[:3,:]
	Jw = Ji[3:,:]
	Ri = sy.eye(3)
	return Jv.T*Mi*Jv + Jw.T*Ri*Ii*Ri.T*Jw


# need a function to return the F from the lagrangian and a list of all the time dependent variables
def sym_lagrangian(link_list_cm, M, I, qdot):
	g = sy.Matrix([[0],[-gravity],[0]])
	[link_list, Ocm] = link_list_cm
	A = sym_get_A0n(link_list)
	R = [Ai[:3,:3] for Ai in A]
	O = [sy.Matrix(Ai[:,3][:3]) for Ai in A]
	J = sym_pt_jacobian(link_list_cm)
	D = symsum([D_i([ J[i], M[i], I[i], R[i]]) for i in range( len(J))])
	K = .5*(qdot.T*(D)*qdot)
	O0c = [A[i]*Ocm[i] for i in range(len(A))]
	P = symsum( [g.T*M[i]*sy.Matrix(O0c[i][:3]) for i in range(len(J))])
	return K-P

def equations_of_motion( L, qvec, qdvec, qddvec, tdv):
	dlist = []
	for qd in qdvec:
		dlist.append(sy.diff(L, qd))
	for i in range(len(dlist)):
		dt = 0
		for j in range(len(tdv)):
			dt = dt + sy.diff(dlist[i], tdv[j][0])*tdv[j][1]
		dlist[i] = dt
		for j in dlist:
			G = L.subs(zip(list(qdvec)+list(qddvec),[0 for jkfjsl in range(len(list(qdvec)+list(qddvec)))]))
			dlist[i] = dlist[i]-sy.diff(G, qvec[i])
	return dlist

#def state_space( eom, qddvec):

def sym_torque(link_list_cm, M, I, qdot, q, tdv_vec):
	L = sym_lagrangian(link_list_cm, M, I, qdot)[0]
	dLdq_dot = sum([sy.diff(L, qdot[i]) for i in range(len(qdot))])
	dLdq =  sum([sy.diff(L, q[i]) for i in range(len(q))])
	ddtdLdq_dot = sum([sy.diff(dLdq_dot, tdv_vec[i][0])*tdv_vec[i][1] for i in range(len(tdv_vec))])
	return ddtdLdq_dot - dLdq

################################################################################
############################ Trajectory Generation #############################
def q_gen(order):
	t = sy.symbols("t")
	q = np.array([ t**i for i in range(order+1)])
	qds = [np.array([sy.diff(val,t,i) for val in q]) for i in range(order)]
	return qds

def mat_builder( order, time):
	t = sy.symbols("t")
	time = np.array(time)
	num_o_eqs = len(time)-1
	q_s = q_gen( order)
	q_list = [q_s for i in range(num_o_eqs)]
	b = []
	for i in range(len(q_list[0])):
		b.append([[val[i], val[i]] for val in q_list])
	print num_o_eqs

