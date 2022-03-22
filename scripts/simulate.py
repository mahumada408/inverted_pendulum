"""
Simulation of controlling the inverted pendulum on a cart with state controller.

Equations:
  x_ddot = (u - (m_p * l * cos(theta) * theta_ddot) + (m_p * l * sin(theta) * theta_dot * theta_dit)) / (m_p + m_c)
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

def PenParams():
  # physical constants
  g = -9.8
  l = 0.3
  m_p = 0.2
  m_c = 0.5
  i_zz = 0.006

  return g, l, m_p, m_c, i_zz

def derivatives(state, t, u):
  g, l, m_p, m_c, i_zz = PenParams()
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

def SimStep(x_in, u):
  # simulation time
  dt = 0.01
  Tmax = 5
  # t = np.arange(0.0, Tmax, dt)
  t = np.array([0.0, 0.01])

  state = np.array([x_in[0,0], x_in[1,0], x_in[2,0], x_in[3,0]])
  # state = np.array([x, x_dot, theta, theta_dot])
  print(state)
  print("Integrating...")
  # integrate your ODE using scipy.integrate.
  solution = integrate.odeint(derivatives, state, t, args=(u,))
  print("Done")

  # ths = solution[:, 2]
  # xs = solution[:, 0]
  # print(solution.shape)

  return np.array([[solution[1,0]], [solution[1,1]], [solution[1,2]], [solution[1,3]]])