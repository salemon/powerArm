import numpy as np
from scipy.optimize import minimize
from scipy.spatial import KDTree
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
import utilities


data = np.load('circle_traj.npz')
q_pos = data['q_pos']
q_vel = data['q_vel']
q_tau = data['q_tau']
wrenchOffset = data['wrenchOffset']

opt = dynamic_optimization()
with open('optimized_params_2.npy', 'rb') as f:
    params = np.load(f)

IDX_TOOL = opt.model.getFrameId('ee_link')

opt.setParameters(params)
robotData = opt.model.createData()
ee_pos = []

for i in range(q_pos.shape[0]):
    pin.framesForwardKinematics(opt.model, robotData, q_pos[i])
    ee_pos.append(np.copy(robotData.oMf[IDX_TOOL].translation))
    


ee_pos = np.array(ee_pos)

# ee_pos = np.random.rand(1000,3) * 10

tree = KDTree(ee_pos)

st = time.time()
d, i = tree.query(np.array([1.08041804, -0.1631816, 0.98235673]))
et = time.time()

print(et - st)
print(len(ee_pos))

print(d, i)

print(max(wrenchOffset[:, 0]) - min(wrenchOffset[:, 0]))
print(max(wrenchOffset[:, 1]) - min(wrenchOffset[:, 1]))
print(max(wrenchOffset[:, 2]) - min(wrenchOffset[:, 2]))
# print(ee_pos)
print("-----------------------------------------------")

# for i in range(4):

file = open('joint_output.txt','r')

# file = open('cartesian_circle_points.txt','r')
traj_pos = []
traj_vel = []
q_vel_list = []

content = file.readline()

while True:
    content = file.readline()
    if not content or len(content) < 9:
        break

    temp = content.split(",")
    traj_pos.append([float(temp[i]) for i in range(4)])
    # q_vel_list.append([float(temp[i]) for i in range(4, 8)])
    # traj_vel.append([round(utilities.rad2rpm(float(temp[i]), i % 4)) for i in range(4, 8)])
traj_pos = np.array(traj_pos)
file.close()



# print(traj_pos.type)
# traj_pos.reshape((traj_pos.shape[0]//4, 4))
# # print(traj_pos.shape[0]/4)

# print(traj_pos.shape)
# traj_pos = np.array(traj_pos)


# with open('optimized_q.npy', 'rb') as f:
#     traj_pos = np.load(f)
# traj_pos = traj_pos.reshape((traj_pos.shape[0]//4, 4))
# print(traj_pos.shape)
# np.savetxt('opt_q.txt', traj_pos, delimiter=',')
# print(traj_pos)

# rpm = [200, 500, 200, 500]
# for i in range(4):
#     print(utilities.rpm2rad(rpm[i], i)/20)


i = 3
for j in range(1, traj_pos.shape[0] + 1):
    # print(traj_pos[j % traj_pos.shape[0]][i]/np.pi * 180)
    # print(traj_pos[j][i] - traj_pos[j - 1][i] + 0.05 * q_vel_list[j - 1][i])
    print(utilities.rad2rpm((traj_pos[j % traj_pos.shape[0]][i] - traj_pos[j - 1][i])/0.05, i), traj_pos[j % traj_pos.shape[0]][i] - traj_pos[j - 1][i])



