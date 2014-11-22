#!/usr/bin/env python

import sys
sys.path.append(r"/Users/robertbrothers/Desktop/Fall 2014/Fundamentals_of_Robotics/robo_git/python/")

import robotics_functions as rf, numpy as np, scipy as sp, sympy as sy
from numpy import cos,sin
from scipy import integrate

import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation

l1,l2,m1,m2,I1,I2 = [.5,.5,2.0,2.0,.01,.01]
g = 9.81

################# STATE SPACE #################
# initial conditions
init_con = [15*np.pi/180,0,0,0]
# time span
dt = .01
t = np.arange(0,20,dt)
def update( x0, ti):
	q1,q2,qdot1,qdot2 = x0
	G1 = -g*(1.0*l1*m1*cos(q1) + 2.0*l1*m2*cos(q1) + 1.0*l2*m2*cos(q1 + q2))
	G2 = -1.0*g*l2*m2*cos(q1 + q2)
	G = np.array([
		[G1],
		[G2],
		])
	C1 = -l1*l2*m2*qdot2*(1.0*qdot1 + 0.5*qdot2)*sin(q2)
	C2 = -0.5*l1*l2*m2*qdot1*qdot2*sin(q2)
	C = np.array([
		[C1],
		[C2],
		])
	M11 = 1.0*I1 + 1.0*I2 + 0.25*l1**2*m1 + 1.0*l1**2*m2 + 1.0*l1*l2*m2*cos(q2) + 0.25*l2**2*m2
	M12 = 1.0*I2 + 0.5*l1*l2*m2*cos(q2) + 0.25*l2**2*m2
	M21 = 1.0*I2 + 0.5*l1*l2*m2*cos(q2) + 0.25*l2**2*m2
	M22 = 1.0*I2 + 0.25*l2**2*m2
	M = np.array([
		[M11,M12],
		[M21,M22],
		])
	print -G-C
	print M
	y0 = np.linalg.solve(M,-G-C)
	x1,x2,y1,y2 = [x0[1],y0[0],x0[3],y0[1]]
	return x1,x2,y1,y2

x0 = sp.integrate.odeint( update, init_con,t)


################################### ANIMATION #################################
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
ax.grid()

line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.01, 0.9, '', transform=ax.transAxes)

x1 = l1*sin(x0[:,0])
y1 = -l1*cos(x0[:,0])

x2 = l2*sin(x0[:,2]) + x1
y2 = -l2*cos(x0[:,2]) + y1

def init():
	line.set_data([], [])
	time_text.set_text('')
	return line, time_text

def animate(i):
	thisx = [0, x1[i], x2[i]]
	thisy = [0, y1[i], y2[i]]

	line.set_data(thisx, thisy)
	time_text.set_text(time_template%(i*dt))
	return line, time_text

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(x0)),
		    interval=25, blit=False, init_func=init)

#ani.save('ein_animation.mp4', fps=15)
plt.show()
