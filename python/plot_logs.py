#!/usr/bin/python

import sys
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt

if len(sys.argv) != 3:
  print 'Usage: ./plot_logs (ekf_log) (improb log)'
  sys.exit()

ekf_data = genfromtxt(sys.argv[1], delimiter=',')
improb_data = genfromtxt(sys.argv[2], delimiter=',')

n = ekf_data.shape[0]

ekf_ploty = np.zeros(n)
ekf_plottheta = np.zeros(n)


improb_ploty = np.zeros(n)
improb_plottheta = np.zeros(n)


for i in range(0, n):
  ekf_ploty[i] = np.sqrt((ekf_data[i,0] - ekf_data[i,3])**2 + (ekf_data[i,1] - ekf_data[i,4])**2)
  improb_ploty[i] = np.sqrt((improb_data[i,0] - improb_data[i,3])**2 + (improb_data[i,1] - improb_data[i,4])**2)
  
  ekf_plottheta[i] = np.abs(ekf_data[i,2] - ekf_data[i,5])
  improb_plottheta[i] = np.abs(improb_data[i,2] - improb_data[i,5])

plt.figure()
plt.xlabel("Timestep")
plt.ylabel("Linear Distance")
plt.title("Measurement - Prediction, X and Y")
plt.plot(range(0,n), ekf_ploty, 'b-', label='EKBF')
plt.plot(range(0,n), improb_ploty, 'r-', label='Improbability')
plt.legend()

plt.figure()
plt.xlabel("Timestep")
plt.ylabel("Abs Theta Difference")
plt.title("Measurement - Prediction, Theta")
plt.plot(range(0,n), ekf_ploty, 'b-', label='EKBF')
plt.plot(range(0,n), improb_ploty, 'r-', label='Improbability')
plt.legend()
plt.show()
