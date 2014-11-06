#!/usr/bin/env python

import robotics_functions as rf, numpy as np, scipy as sp, sympy as sy

[l1, l2, l3, t1, t2, t3, a1, a2, a3, d1, d2, d3] = sy.symbols("l1 l2 l3 t1 t2 t3 a1 a2 a3 d1 d2 d3")

link_list_pt = [[                                                                                       
		[a1, 0, 0, t1],
		[a2, 0, 0, t2]],
		[ sy.Matrix([[-a1/2],[0],[0],[1]]),
			sy.Matrix([[-a2/2],[0],[0],[1]])]
		]

link_list_cm = [                                                                                       
		[a1/2, 0, 0, t1],
		[a2/2, 0, 0, t2]]

if __name__ == "__main__":
	print rf.sym_pt_jacobian(link_list_pt)
