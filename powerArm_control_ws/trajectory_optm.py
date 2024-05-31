import numpy as np
from scipy.optimize import minimize, least_squares
import pinocchio as pin
from sys import argv
import pandas as pd
from os.path import dirname, join, abspath
import utilities


class trajectory_optimization():
    def __init__(self) -> None:
        pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "powerArm_control_ws")
        urdf_filename = pinocchio_model_dir + '/powerarm_urdf.urdf' if len(argv)<2 else argv[1]
        self.model = pin.buildModelFromUrdf(urdf_filename)
        self.num_links = len(self.model.inertias)
        self.IDX_TOOL = self.model.getFrameId('ee_link')

        self.num_ite = 0

        self.readData()

        self.bnds = [(-np.pi/2, np.pi/4), 
                     (-np.pi/6, np.pi/2),
                     (-np.pi/6, np.pi/6),
                     (0, np.pi/2)] * self.q_pos.shape[0]

    def readData(self):
        file = open('cartesian_circle_points.txt','r')
        # file = open('circle_points.txt','r')
        self.q_pos = []
        self.motor_vel = []
        self.q_vel = []
        self.traj_pos = []

        data = self.model.createData()

        content = file.readline()

        while True:
            content = file.readline()
            if not content or len(content) < 9:
                break

            temp = content.split(",")
            self.q_pos.append([float(temp[i]) for i in range(4)])
            self.q_vel.append([float(temp[i]) for i in range(4, 8)])
            pin.framesForwardKinematics(self.model, data, np.array(self.q_pos[-1]))
            pos = data.oMf[self.IDX_TOOL].translation.copy()
            self.traj_pos.append(pos)
            self.motor_vel.append([round(utilities.rad2rpm(float(temp[i]), i % 4)) for i in range(4, 8)])

        file.close()

        self.q_pos = np.array(self.q_pos)
        self.q_vel = np.array(self.q_vel)
        self.motor_vel = np.array(self.motor_vel)
        self.traj_pos = np.array(self.traj_pos)    
    
    def computeModelPredictions(self, q):
        data = self.model.createData()
        pin.forwardKinematics(self.model, data, q)
        pin.framesForwardKinematics(self.model, data, q)

        pos = data.oMf[self.IDX_TOOL].translation.copy()

        return pos
    
    def setConstraints(self):
        self.constraints = []
        n = len(self.q_pos)
        v_bound = [0.04, 0.1, 0.008, 0.06]
        # print(n)
        # for i in range(n):
        #     # for j in range(4):
        #     self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: q[4 * ((i + 1) % n) + 0] - q[4 * i + 0] + 0.01})
        #     self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: -q[4 * ((i + 1) % n) + 0] + q[4 * i + 0] + 0.01})
        #     self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: q[4 * ((i + 1) % n) + 1] - q[4 * i + 1] + 0.01})
        #     self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: -q[4 * ((i + 1) % n) + 1] + q[4 * i + 1] + 0.01})
        #     self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: q[4 * ((i + 1) % n) + 2] - q[4 * i + 2] + 0.005})
        #     self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: -q[4 * ((i + 1) % n) + 2] + q[4 * i + 2] + 0.005})
        #     self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: q[4 * ((i + 1) % n) + 3] - q[4 * i + 3] + 0.004})
        #     self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: -q[4 * ((i + 1) % n) + 3] + q[4 * i + 3] + 0.004})

        for i in range(n - 1):
            # for j in range(4):
            self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: q[4 * (i + 1) + 0] - q[4 * i + 0] + v_bound[0]})
            self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: -q[4 * (i + 1) + 0] + q[4 * i + 0] + v_bound[0]})
            self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: q[4 * (i + 1) + 1] - q[4 * i + 1] + v_bound[1]})
            self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: -q[4 * (i + 1) + 1] + q[4 * i + 1] + v_bound[1]})
            self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: q[4 * (i + 1) + 2] - q[4 * i + 2] + v_bound[2]})
            self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: -q[4 * (i + 1) + 2] + q[4 * i + 2] + v_bound[2]})
            self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: q[4 * (i + 1) + 3] - q[4 * i + 3] + v_bound[3]})
            self.constraints.append({'type': 'ineq', 'fun': lambda q, i=i: -q[4 * (i + 1) + 3] + q[4 * i + 3] + v_bound[3]})

        self.constraints.append({'type': 'eq', 'fun': lambda q: q[4 * (n - 1) + 0] - q[0]})
        self.constraints.append({'type': 'eq', 'fun': lambda q: q[4 * (n - 1) + 1] - q[1]})
        self.constraints.append({'type': 'eq', 'fun': lambda q: q[4 * (n - 1) + 2] - q[2]})
        self.constraints.append({'type': 'eq', 'fun': lambda q: q[4 * (n - 1) + 3] - q[3]})

        # self.constraints.append({'type': 'ineq', 'fun': lambda q: q[4 * (n - 1) + 0] - q[0] + 0.005})
        # self.constraints.append({'type': 'ineq', 'fun': lambda q: -q[4 * (n - 1) + 0] + q[0] + 0.005})
        # self.constraints.append({'type': 'ineq', 'fun': lambda q: q[4 * (n - 1) + 1] - q[1] + 0.005})
        # self.constraints.append({'type': 'ineq', 'fun': lambda q: -q[4 * (n - 1) + 1] + q[1] + 0.005})
        # self.constraints.append({'type': 'ineq', 'fun': lambda q: q[4 * (n - 1) + 2] - q[2] + 0.005})
        # self.constraints.append({'type': 'ineq', 'fun': lambda q: -q[4 * (n - 1) + 2] + q[2] + 0.005})
        # self.constraints.append({'type': 'ineq', 'fun': lambda q: q[4 * (n - 1) + 3] - q[3] + 0.005})
        # self.constraints.append({'type': 'ineq', 'fun': lambda q: -q[4 * (n - 1) + 3] + q[3] + 0.005})


    def objectiveFunction(self, q):
        
        # Compute model predictions
        pos_pred = np.zeros((self.traj_pos.shape[0], 3))
        n = self.traj_pos.shape[0]
        prev_q = q[4 * (n - 1) : 4 * (n - 1) + 4]
        effort = 0
        w = 50

        for i in range(self.traj_pos.shape[0]):
            cur_q = q[4 * i : 4 * i + 4]
            effort += np.sum((prev_q[2] - cur_q[2])**2)

            pos_pred[i] = self.computeModelPredictions(cur_q)
            prev_q = cur_q

        error = (np.sum((pos_pred - self.traj_pos)**2) + w * effort)/ self.q_pos.shape[0]
        print(error)
        return error
    

    def callback(self, xk, *_):
        self.num_ite += 1
        print("Number of Iteration: ", self.num_ite)

        # for i in range(self.num_links):
        #     print("mass[%d]: " % (i))
        #     print(xk[i * 10])
        #     print("lever[%d]: " % (i))
        #     print(xk[i * 10 + 1 : i * 10 + 4])

        #     temp_inertias = np.zeros((3, 3))
        #     temp_inertias[0] = xk[i * 10 + 4 : i * 10 + 7]
        #     temp_inertias[1, 1:] = xk[i * 10 + 7 : i * 10 + 9]
        #     temp_inertias[2, 2] = xk[i * 10 + 9]
        #     print("inertias[%d]" % (i))
        #     print(temp_inertias + temp_inertias.T - np.select([temp_inertias == np.diag(temp_inertias)], [temp_inertias], default=0))



opt = trajectory_optimization()
opt.readData()
# # print(opt.traj_pos)
# # init_params = np.zeros(opt.num_links * 10 + 4)
init_params = opt.q_pos.flatten()
opt.setConstraints()

# opt.computeModelPredictions(init_params)

# print(opt.q.shape)
# print(opt.expTau.shape)

# res = minimize(opt.objectiveFunction, init_params, method='nelder-mead', callback = opt.callback, bounds=opt.bnds, options={'xatol': 1e-3, 'disp': True})
# res = minimize(opt.objectiveFunction, init_params, method='SLSQP', callback = opt.callback, bounds=opt.bnds, constraints=opt.constraints, options={'maxiter':728, 'disp': True, 'return_all': True})
res = minimize(opt.objectiveFunction, init_params, method = 'SLSQP', callback = opt.callback, bounds=opt.bnds, constraints=opt.constraints, options={'ftol': 1e-7, 'disp': True})

# res = minimize(opt.objectiveFunction, init_params, method='TNC', callback = opt.callback, options={'disp': True})

# res = minimize(opt.objectiveFunction, init_params, method='Powell', callback = opt.callback, bounds=opt.bnds, options={'disp': True})

# res = least_squares(opt.objectiveFunction, init_params)


# with open('optimized_params.npy', 'wb') as f:
#     np.save(f, res.x)

sol = res.x.reshape((opt.q_pos.shape[0], 4))
print(sol)

with open('optimized_q.npy', 'wb') as f:
    np.save(f, res.x)

print(res)
# result.set_parameters()
# print(*result.model.inertias)