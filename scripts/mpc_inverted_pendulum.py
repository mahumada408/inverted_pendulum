import sys
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '/home/manuel/projects/inverted_pendulum/third_party/rtPGM')

from controller import rtPGM
from controller_codegen import *
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import cont2discrete
import random
import simulate

# System parameters
m_p = 0.2 # kg
m_c = 0.5 # kg
l = 0.3 # m -- distance to cg
izz = 0.006 # kg * m^2
b = 0.0
g = 9.81 # m/s^2

# den = (izz * (m_c + m_p)) + (m_c * m_p * l * l)

den = ((izz + m_p * l * l) * (m_c + m_p)) - (m_p * m_p * l * l)

Ac = np.array([[0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, (m_p * m_p * g * l * l) / den, 0.0],
               [0.0, 0.0, 0.0, 1.0],
               [0.0, 0.0, (m_p * g * l * (m_c + m_p)) / den, 0.0]])
Bc = np.array([[0.0],
               [(izz + m_p * l * l) / den],
               [0.0],
               [(m_p * l) / den]])

Cc = np.array([[1.0, 0.0, 0.0, 0.0]])
Dc = np.array([[0.0]])

t_s = 0.01

A, B, C, D, _ = cont2discrete([Ac, Bc, Cc, Dc], t_s, method='zoh')

print(C)
print(D)

umin = -200.0
umax = 200.0
nx = A.shape[0]
nq = B.shape[1]

horizon_time = 1.0 # s
N = int(horizon_time / t_s)

# controller
Q = np.array([[100.0, 0.0, 0.0, 0.0],
              [0.0, 1.0, 0.0, 0.0],
              [0.0, 0.0, 1000.0, 0.0],
              [0.0, 0.0, 0.0, 100.0]])
R = 1000.0 * np.eye(nq)

print(A.shape)
print(B.shape)
print(C.shape)
print(D.shape)
print(Q.shape)
print(R.shape)
controller = rtPGM(A, B, C, D, Q, R, umin, umax, N)
controller.codegen("/home/manuel/projects/inverted_pendulum/mbed/library/inverted_pendulum.h")

sim_time = 20 # s
n_sim = int(sim_time / t_s)

q = np.zeros((N, 1))
x = np.array([[0.], [0.], [0.01], [0.0]]) # initial condition
r = np.array([[0.0]])     # terminal state
x_sv = np.zeros((nx, 0)) # all states
q_sv = np.zeros((nq, 0)) # all control inputs
VN_sv = np.zeros((1, 0))

print(x[0,])

for k in range(n_sim):
    x_sv = np.hstack((x_sv, x))
    # update trajectory
    q = controller.update(x, r)
    q_sv = np.hstack((q_sv, q))
    # update system
    # x = A.dot(x) + B.dot(q)
    x[2,] += np.pi
    x = simulate.SimStep(x, -q)
    x[2,] -= np.pi
    # if k % int( 1 / t_s) == 0:
    #     x[2] += random.randrange(-1.0, 1.0)

# plot
time = np.linspace(0, (n_sim-1) * t_s, n_sim)
plt.figure()
plt.subplot(3, 1, 1)
plt.plot(time, x_sv[0, :], 'b')
plt.legend(['x'])
plt.plot([0, (n_sim-1) * t_s], [r[0, 0], r[0, 0]], 'r--')
plt.xlim([0, (n_sim-1) * t_s])
plt.ylabel('x')
plt.subplot(3, 1, 2)
plt.plot(time, x_sv[2, :], 'b')
plt.legend(['theta'])
plt.xlim([0, (n_sim-1) * t_s])
plt.ylabel('theta')
plt.subplot(3, 1, 3)
plt.plot(time, q_sv[0, :], 'b')
plt.legend(['Fc'])
plt.xlim([0, (n_sim-1) * t_s])
plt.ylabel('Fc (N)')
plt.show()