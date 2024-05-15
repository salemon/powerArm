import numpy as np
from scipy.optimize import minimize, least_squares
import pinocchio as pin
from sys import argv
import pandas as pd
from os.path import dirname, join, abspath


class dynamic_optimization():
    def __init__(self) -> None:
        pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "powerArm_control_ws")
        urdf_filename = pinocchio_model_dir + '/powerarm_urdf.urdf' if len(argv)<2 else argv[1]
        self.model = pin.buildModelFromUrdf(urdf_filename)
        self.num_links = len(self.model.inertias)

        self.num_ite = 0

        self.readData()
        self.bnds = [(None, None)] * (self.num_links * 10 + 4)

    def readData(self):
        self.q = []
        self.qv = np.zeros(4)
        self.expTau = []

        # df = pd.read_excel('data_w_6axis_torque_sensor.xlsx')
        df = pd.read_excel('raw_data_3_29.xlsx')
        # df = pd.read_excel('ARM_234_joint.xlsx')
        self.num_data = df.shape[0]

        for i in range(self.num_data):
            # if df['J4 torque (Nm)'][i] < 0:
                self.q += [[90, df['J2 angle (deg)'][i], df['J3 angle (deg)'][i], df['J4 angle (deg)'][i]]]
                self.expTau += [[0, df['J2 torque (Nm)'][i], df['J3 torque (Nm)'][i], df['J4 torque (Nm)'][i]]]

        self.q = np.array(self.q)
        self.q = self.robot2SimRad(self.q)

        self.expTau = np.array(self.expTau) * np.array([1, -1, -1, 1])
        
        # print(self.q.shape)
        # print(self.expTau.shape)
        
    def model2Params(self, params):
        for i in range(0, self.num_links):
            params[i * 10] = self.model.inertias[i].mass
            params[i * 10 + 1 : i * 10 + 4] = self.model.inertias[i].lever
            params[i * 10 + 4 : i * 10 + 7] = self.model.inertias[i].inertia[0, :]
            params[i * 10 + 7 : i * 10 + 9] = self.model.inertias[i].inertia[1, 1:]
            params[i * 10 + 9 : i * 10 + 10] = self.model.inertias[i].inertia[2, 2:]

                   # for i in range(self.num_links):
            self.bnds[i * 10] = (self.model.inertias[i].mass * 0.5, self.model.inertias[i].mass * 3)

        return params
        
    def setParameters(self, params):
        for i in range(0, self.num_links):
            self.model.inertias[i].mass = params[i * 10]
            self.model.inertias[i].lever = params[i * 10 + 1 : i * 10 + 4]

            temp_inertias = np.zeros((3, 3))
            temp_inertias[0] = params[i * 10 + 4 : i * 10 + 7]
            temp_inertias[1, 1:] = params[i * 10 + 7 : i * 10 + 9]
            temp_inertias[2, 2] = params[i * 10 + 9]

            self.model.inertias[i].inertia = (temp_inertias + temp_inertias.T - np.select([temp_inertias == np.diag(temp_inertias)], [temp_inertias], default=0))

    def computeModelPredictions(self, q, qv):
        data = self.model.createData()
        pin.forwardKinematics(self.model, data, q)
        pin.framesForwardKinematics(self.model, data, q)

        b = pin.nle(self.model, data, q, qv)

        return b

    def robot2SimRad(self, q):
        qr = (q + np.array([-90, -90, -90, 0])) * np.array([1, -1, 1, 1])
        qr = qr / 180 * np.pi

        return qr

    def objectiveFunction(self, params):
        self.setParameters(params)
        
        # Compute model predictions
        # model_predictions = np.zeros((self.num_data, 4))
        model_predictions = np.zeros((self.q.shape[0], 4))

        # for i in range(self.num_data):
        for i in range(self.q.shape[0]):
            model_predictions[i] = self.computeModelPredictions(self.q[i], self.qv) + params[-4:]
            # model_predictions[i] = self.computeModelPredictions(self.q[i], self.qv)

        # print(model_predictions)

        # print(min(model_predictions[:, 3]), max(model_predictions[:, 3]))

        error = np.sum((self.expTau[:, :3] - model_predictions[:, :3])**2) / self.num_data
        print(error)
        return error


    def callback(self, xk, *_):
        self.num_ite += 1
        print("Number of Iteration: ", self.num_ite)

        for i in range(self.num_links):
            print("mass[%d]: " % (i))
            print(xk[i * 10])
            print("lever[%d]: " % (i))
            print(xk[i * 10 + 1 : i * 10 + 4])

            temp_inertias = np.zeros((3, 3))
            temp_inertias[0] = xk[i * 10 + 4 : i * 10 + 7]
            temp_inertias[1, 1:] = xk[i * 10 + 7 : i * 10 + 9]
            temp_inertias[2, 2] = xk[i * 10 + 9]
            print("inertias[%d]" % (i))
            print(temp_inertias + temp_inertias.T - np.select([temp_inertias == np.diag(temp_inertias)], [temp_inertias], default=0))

# opt = dynamic_optimization()
# init_params = np.zeros(opt.num_links * 10 + 4)
# init_params = opt.model2Params(init_params)

# print(opt.q.shape)
# print(opt.expTau.shape)

# res = minimize(opt.objectiveFunction, init_params, method='nelder-mead', callback = opt.callback, bounds=opt.bnds, options={'xatol': 1e-3, 'disp': True})
# res = minimize(opt.objectiveFunction, init_params, method='BFGS', callback = opt.callback, options={'xrtol': 1e-6, 'disp': True, 'return_all': True})

# res = minimize(opt.objectiveFunction, init_params, method='TNC', callback = opt.callback, options={'disp': True})

# res = minimize(opt.objectiveFunction, init_params, method='Powell', callback = opt.callback, bounds=opt.bnds, options={'disp': True})

# res = least_squares(opt.objectiveFunction, init_params)

# print(res.x)
# with open('optimized_params.npy', 'wb') as f:
#     np.save(f, res.x)


# result.set_parameters()
# print(*result.model.inertias)