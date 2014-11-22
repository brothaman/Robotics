#!/usr/bin/env python

import sys
sys.path.append(r"/Users/robertbrothers/Desktop/Fall 2014/Fundamentals_of_Robotics/robo_git/python/")
from matplotlib.backends.backend_pdf import PdfPages
import robotics_functions as rf, sympy as sy, numpy as np
from matplotlib import pyplot as plt
pp = PdfPages('homework10plots.pdf')

order = 3

Q = lambda t: np.array([[1, t, t**2, t**3], [0, 1, 2*t, 3*t**2], [0, 0, 2, 6*t]])
a = np.array([10,5,7,-1]).T

time = [0.0,1.0]
Q = [Q(t)[:2] for t in time]
Q = np.vstack((Q[0][0],Q[1][0],Q[0][1],Q[1][1]))

#print np.matrix(Q)*np.matrix(a).T

# Question 2 using the same 
ini_con = np.matrix([0.0,5.0,0.0,0.0])
time = [0.0,2.0]
Q = lambda t: np.array([[1, t, t**2, t**3], [0, 1, 2*t, 3*t**2], [0, 0, 2, 6*t]])
Q = [Q(t)[:2] for t in time]
Q = np.matrix(np.vstack((Q[0][0],Q[1][0],Q[0][1],Q[1][1])))
a = np.linalg.solve(Q,ini_con.T)
#print a

time = np.arange(0.0,2.0, 2.0/100)
theta = lambda t: 0+0*t+3.75*t**2-1.25*t**3
thetadot = lambda t: 0+ 1*0.0+ 3.75*2*t - 1.25*3*t**2
thetaddot = lambda t: 0 + 0 + 3.75*2 - 1.25*6*t
plt.plot(time,theta(time[:]),'r', label='position',)
plt.plot(time,thetadot(time[:]),'b',label='velocity',)
plt.plot(time,thetaddot(time[:]),'g',label='acceleration')

plt.xlabel("time")
plt.ylabel(r"$\theta$, $\dot{\theta}$ and $\ddot{\theta}$")

plt.title(r"Question 2: $\theta$, $\dot{\theta}$ and $\ddot{\theta}$ vs. time")
plt.legend(loc=3)
plt.savefig(pp, format='pdf')
plt.show()


#### question 3
qi = np.array([ 0,1,1,0, 0,0,0,0 ])
ti = [0,1.0,1.0,3.0]
q01 = lambda t: np.array([1,t,t**2,t**3,  0,0,0,0])
qd01 = lambda t: np.array([0,1,2*t,3*t**2, 0,0,0,0])

q12 = lambda t: np.array([0,0,0,0, 1,t,t**2,t**3])
qd12 = lambda t: np.array([ 0,0,0,0, 0,1,2*t,3*t**2])

Q = np.array([
	q01(ti[0]),
	q01(ti[1]),
	q12(ti[2]),
	q12(ti[3]),
	qd01(ti[0]),
	qd01(ti[1]),
	qd12(ti[2]),
	qd12(ti[3]),
	])

a = np.linalg.solve(Q, qi.T)
print a
[a10,a11,a12,a13,   a20,a21,a22,a23 ] = a
#

qi 		= lambda a,t: a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3
qdi		= lambda a,t: a[1] + 2*a[2]*t**2 + 3*a[3]*t**2
qddi	= lambda a,t: 2*a[2] + 6*a[3]*t

time1 = np.arange(0,1.0,1.0/100)
time2 = np.arange(1.0,3.0,1.0/100)

plt.plot(time1,qi( a[0:4], time1[:]),'r', label='position 1',linewidth=3)
plt.plot(time2,qi( a[4:], time2[:]),'r', label='position 2',)

plt.xlabel("time")
plt.ylabel(r"$\theta$")
plt.title(r"Question 3: $\theta$ vs. time")
plt.legend(loc=3)
plt.savefig(pp, format='pdf')
plt.show()


plt.plot(time1,qdi( a[0:4], time1[:]),'b',label='velocity 1',linewidth=3)
plt.plot(time2,qdi( a[4:], time2[:]),'b',label='velocity 2',)

plt.xlabel("time")
plt.ylabel(r"$\dot{\theta}$")
plt.title(r"Question 3: $\dot{\theta}$ vs. time")
plt.legend(loc=3)
plt.savefig(pp, format='pdf')
plt.show()

plt.plot(time1,qddi( a[0:4], time1[:]),'m',label='acceleration 1',linewidth=3)
plt.plot(time2,qddi( a[4:], time2[:]),'m',label='acceleration 2')

plt.xlabel("time")
plt.ylabel(r"$\ddot{\theta}$")
plt.title(r"Question 3: $\ddot{\theta}$ vs. time")
plt.legend(loc=3)
plt.savefig(pp, format='pdf')
plt.show()
#plt.savefig("question3.png")
pp.close()
