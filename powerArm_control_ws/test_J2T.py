import numpy as np
from scipy.optimize import minimize
import pinocchio as pin
from sys import argv
import pandas as pd
from numpy.linalg import pinv,inv,norm,svd,eig
from os.path import dirname, join, abspath
from optimization import dynamic_optimization
import time
import queue
import serial

opt = dynamic_optimization()

with open('optimized_params.npy', 'rb') as f:
    params = np.load(f)

IDX_TOOL = opt.model.getFrameId('ee_link')

opt.setParameters(params)
robotData = opt.model.createData()
q = np.array([0, 60, 0, 10]) / 180 * np.pi

pin.forwardKinematics(opt.model, robotData, q)
pin.framesForwardKinematics(opt.model, robotData, q)

frame_pos = robotData.oMf[IDX_TOOL].translation
print("org_frame: ", frame_pos)

# q = np.array([-0.2922597,   0.77033391, -0.01371135,  0.97328736])
# print(q /np.pi * 180)
# wrench = np.array([11.7,   -3.13,  -3.83,  0.218,  0.91,  -0.045])
# Jtool = pin.computeFrameJacobian(opt.model, robotData, q, IDX_TOOL)
# tau = Jtool.T @ wrench

# print(tau)


# DT = 1e-2
# for i in range(500):
#     pin.framesForwardKinematics(opt.model, robotData, q)
#     pin.computeJointJacobians(opt.model, robotData, q)
    
#     oMtool = robotData.oMf[IDX_TOOL]
#     print(oMtool)
#     print(oMtool.inverse())
#     Jtool = pin.computeFrameJacobian(opt.model, robotData, q, IDX_TOOL)
#     print(Jtool)

#     o_TG = oMtool - np.array([0.1, -0.1, 0, 0, 0, 0])
#     qdot = -pinv(Jtool) @ o_TG

#     q = pin.integrate(opt.model, q, qdot * DT)

#     print(o_TG)
