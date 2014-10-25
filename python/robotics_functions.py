import numpy as np, sympy as sy

#define numerical rotation matricies
def rotation_z(theta):
	return np.matrix([
		[np.cos(theta), -np.sin(theta), 0],
		[np.sin(theta),  np.cos(theta), 0],
		[0, 0, 1]
		])
def rotation_y(theta):
	return np.matrix([
		[np.cos(theta), 0, -np.sin(theta)],
		[0, 1, 0],
		[np.sin(theta), 0,  np.cos(theta)]
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
		[sy.cos(theta), -sy.sin(theta), 0],
		[sy.sin(theta),  sy.cos(theta), 0],
		[0, 0, 1]
		])
def sym_rotation_y(theta):
	return sy.Matrix([
		[sy.cos(theta), 0, -sy.sin(theta)],
		[0, 1, 0],
		[sy.sin(theta), 0,  sy.cos(theta)]
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

def denavit_hartenberg( link_list):
	return (
		translation_z(link_list[0])*convert3x3to4x4(rotation_z(link_list[1]))*
		translation_x(link_list[2])*convert3x3to4x4(rotation_x(link_list[3]))
		)
def sym_denavit_hartenberg( link_list):
	return (
		sym_translation_z(link_list[0])*sy.Matrix(syms_convert3x3to4x4(sym_rotation_z(link_list[1])))*
		sym_translation_x(link_list[2])*sy.Matrix(syms_convert3x3to4x4(sym_rotation_x(link_list[3])))
		)


