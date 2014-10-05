
import numpy as np

def end_effector( list_of_links):
	number_of_links = len(list_of_links)
	A_list = [ A_matrix(val) for val in list_of_links]
	val = 1
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



