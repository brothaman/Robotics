
import numpy as np, sympy as sy

def end_effector( list_of_links):
	number_of_links = len(list_of_links)
	whatami = 0
	for val in list_of_links:
		for thing in val:
			if str(type(thing)) == "<class 'sympy.core.symbol.Symbol'>":
				whatami = 1
	if whatami == 0:
		A_list = [ A_matrix(val) for val in list_of_links]
	else:
		A_list = [ A_sym_matrix(val) for val in list_of_links]
	val = np.matrix(np.identity(4))
	for i in A_list:
		val = val*i
	return val

def A_matrix(link_list):
	[a, al, d, th] = link_list
	A_mat = np.matrix((
		(np.cos(th), -np.sin(th)*np.cos(al), np.sin(th)*np.sin(al), a*np.cos(th)), 
		(np.sin(th),  np.cos(th)*np.cos(al),-np.cos(th)*np.sin(al), a*np.sin(th)), 
		(0         ,  np.sin(al)           , np.cos(al)           , d           ), 
		(0         ,  0                    , 0                    , 1           ), 
		))
	return A_mat

def A_sym_matrix(link_list):
	[a, al, d, th] = link_list
	A_mat = np.matrix((
		(sy.cos(th), -sy.sin(th)*sy.cos(al), sy.sin(th)*sy.sin(al), a*sy.cos(th)), 
		(sy.sin(th),  sy.cos(th)*sy.cos(al),-sy.cos(th)*sy.sin(al), a*sy.sin(th)), 
		(0         ,  sy.sin(al)           , sy.cos(al)           , d           ), 
		(0         ,  0                    , 0                    , 1           ), 
		))
	return A_mat



