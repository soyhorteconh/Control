#!/usr/bin/env python
from scipy import signal
import matplotlib.pyplot as plt
import numpy as np
from pylab import *

m = 1
M = 5
L = 2
g = -9.81
d = 1
b = -1

A = np.array([[0, 1, 0, 0],
              [0, -d/M, (b*m*g)/M, 0],
              [0, 0, 0, 1],
              [0, (-b*d)/(M*L), -(b*(m+M)*g)/(M*L), 0]])

B = np.array([[0],
             [1/M],
             [0],
             [b/(M*L)]])

C = np.array([1, 0, 0, 0])

D = np.array([0])

sys = signal.StateSpace(A ,B, C, D)
f = logspace(-2, 2)
w = 2 * pi * f
w, mag, phase = signal.bode(sys,w)

plt.figure()
plt.semilogx(w, mag)    # Bode magnitude plot
plt.title("Bode magnitude plot")
plt.grid(which='both', axis='both')
plt.ylim([-60, 3])

plt.figure()
plt.semilogx(w, phase)  # Bode phase plot
plt.title("Bode phase plot")
plt.grid(which='both', axis='both')

plt.show()