"""
Simulation of controlling the inverted pendulum on a cart with state controller.

Equations:
  x_ddot = (u - (m_p * l * cos(theta) * theta_ddot) + (m_p * l * sin(theta) * theta_dot * theta_dot)) / (m_p + m_c)
  theta_ddot = ((-m_p * l * cos(theta) * x_ddot) + (m_p * g * l * sin(theta))) / (i_zz + (m_p * l * l))

State: 
  [x, x_dot, theta, theta_dot]
"""

import numpy as np
import math

import matplotlib
matplotlib.use('TKAgg')

import matplotlib.pyplot as pp
import scipy.integrate as integrate
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

from math import pi, trunc
from numpy import sin, cos


# physical constants
g = -9.8
l = 0.3
m_p = 0.2
m_c = 0.5
i_zz = 0.006

# simulation time
dt = 0.01
Tmax = 5
t = np.arange(0.0, Tmax, dt)
# t = np.array([0.0, 0.001])

# initial conditions
x = 0.0
x_dot = 0.0
theta = np.pi / 2
theta_dot = 0.0

precision = 0.006

state = np.array([x, x_dot, theta, theta_dot])

def derivatives(state, t, u):
  ds = np.zeros_like(state)

  x = state[0]
  x_dot = state[1]
  theta = state[2]
  theta_dot = state[3]

  # Derivative states
  ds[0] = x_dot
  ds[1] = (u - (m_p * l * math.cos(theta) * ds[3]) + (m_p * l * math.sin(theta) * theta_dot * theta_dot)) / (m_p + m_c)
  ds[2] = theta_dot
  ds[3] = ((-m_p * l * math.cos(theta) * ds[1]) + (m_p * g * l * math.sin(theta))) / (i_zz + (m_p * l * l))

  return ds

u = 0.0
print("Integrating...")
# integrate your ODE using scipy.integrate.
solution = integrate.odeint(derivatives, state, t, args=(u,))
print("Done")

ths = solution[:, 2]
xs = solution[:, 0]

pxs = l * sin(ths) + xs
pys = -l * cos(ths)

fig = pp.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1.0, 1.0), ylim=(-0.5, 0.5))
ax.set_aspect('equal')
ax.grid()

patch = ax.add_patch(Rectangle((0, 0), 0, 0, linewidth=1, edgecolor='k', facecolor='g'))

line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

cart_width = 0.1
cart_height = 0.1

def init():
    line.set_data([], [])
    time_text.set_text('')
    patch.set_xy((-cart_width/2, -cart_height/2))
    patch.set_width(cart_width)
    patch.set_height(cart_height)
    return line, time_text, patch


def animate(i):
    thisx = [xs[i], pxs[i]]
    thisy = [0, pys[i]]

    line.set_data(thisx, thisy)
    time_text.set_text(time_template % (i*dt))
    patch.set_x(xs[i] - cart_width/2)
    return line, time_text, patch

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(solution)),
                              interval=25, blit=True, init_func=init)

pp.show()

# # Set up formatting for the movie files
# print("Writing video...")
# Writer = animation.writers['imagemagick']
# writer = Writer(fps=25, metadata=dict(artist='Sergey Royz'), bitrate=1800)
# ani.save('controlled-cart.gif', writer=writer)