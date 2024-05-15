import numpy as np
from scipy.optimize import minimize
import pinocchio as pin
from sys import argv
import pandas as pd
from numpy.linalg import pinv,inv,norm,svd,eig
from os.path import dirname, join, abspath
from optimization import dynamic_optimization
from nearest_point import nearest_point
import time
import queue
import serial

with open('wrench_offset.npy', 'rb') as f:
    traj_tau = np.load(f)

print(traj_tau)