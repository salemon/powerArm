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

TOP = '0'
START = '2'
HOME = '1'
HALT = '3'

VEL = '1'
POS = '2'
TORQ = '3'

CPR = 3600
GR = [26.3, 26.3, 42.6*3, 42.6]

encOffset = [157, -270, 11000, -9300]  ## 4. 115020 -- 105720
tauOffset = [1.5, 0, 0, 0.16] 
TorqConst = [0, 7, 28, 100]

initial_pos = [50, 90, -82, 40]

def to_char(value):
    chr_val = str(abs(value))
    num_zero = 6 - len(chr_val)
    zero_ph = num_zero * "0"

    if value < 0:
        return "-" + zero_ph + chr_val
    else:
        return "+" + zero_ph + chr_val
    
def rad2rpm(value, motor_num):
    return value / np.pi * 180 * GR[motor_num] / 360 * 60

def rpm2rad(value, motor_num):
    return value * np.pi / 180 / GR[motor_num] * 360 / 60

def enc2deg(value, motor_num):
    return value / CPR / GR[motor_num] * 360

def deg2enc(value, motor_num):
    return round(value * CPR * GR[motor_num] / 360)

def torq2curr(value, motor_num):
    return value * TorqConst[motor_num]


serial_port = '/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_002900343532511431333430-if02'
ser = serial.Serial(serial_port, baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS, timeout=0.5)
print("connected to: " + ser.portstr)
junk = ser.readall()
print(junk)
ser.close()
print("Junk clear")

ser = serial.Serial(serial_port, baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS)
print("connected to: " + ser.portstr)

opt = dynamic_optimization()

with open('optimized_params_2.npy', 'rb') as f:
    params = np.load(f)

with open('circle_tau.npy', 'rb') as f:
    traj_tau = np.load(f)

file = open('circle_points.txt','r')
traj_pos = []
traj_vel = []

content = file.readline()

while True:
    content = file.readline()
    if not content or len(content) < 9:
        break

    temp = content.split(",")
    traj_pos.append([float(temp[i]) for i in range(4)])
    traj_vel.append([round(rad2rpm(float(temp[i]), i % 4)) for i in range(4, 8)])

file.close()

traj_pos = np.array(traj_pos)
traj_vel = np.array(traj_vel)

IDX_TOOL = opt.model.getFrameId('ee_link')

opt.setParameters(params)
robotData = opt.model.createData()


test_posj = np.array([[90, 5, -90, 0]])

try:
    i = 0
    command = '#' + START + POS + to_char(deg2enc(initial_pos[0], 0) + encOffset[0]) \
                                + to_char(deg2enc(initial_pos[1], 1) + encOffset[1]) \
                                + to_char(deg2enc(initial_pos[2], 2) + encOffset[2]) \
                                + to_char(deg2enc(initial_pos[3], 3) + encOffset[3]) + "\r"
    print(command)
    ser.write(command.encode())

    st = time.time()
    ct = time.time()
    q = queue.Queue()
    a = [50,50,50,50] # max motor torq acc
    mt = [100, 260, 1200, 300] # motor torq max
    while ct - st < 3:
        ct = time.time()
        line = ser.readline()
        data = line.decode("utf-8")
        data_ls = data.split(",")
        print(data_ls)
        raw_tau = []

        fx = int(data_ls[0])/100.0
        fy = int(data_ls[1])/100.0
        fz = int(data_ls[2])/100.0

        tx = int(data_ls[3])/1000.0
        ty = int(data_ls[4])/1000.0
        tz = int(data_ls[5])/1000.0

        for i in range(6,10):
            axis_d = data_ls[i].split(" ")
            raw_tau.append(float(axis_d[2]))
        print(raw_tau)
        q.put(np.array(raw_tau))

        if q.qsize() > 3:
            q.get()

        q_sum = np.array([0.0, 0.0, 0.0, 0.0])
        for item in list(q.queue):
            q_sum += item

        prev_tau = q_sum / 3

    wrenchOffset = np.array([-fy, fx, fz, -ty, tx, tz])

    print(prev_tau)
    print("Calibration Done")

    spo_torq = 0
    raw_q = np.array([0.0, 0.0, 0.0, 0.0])
    raw_q_dot = np.array([0.0, 0.0, 0.0, 0.0])
    raw_tau = np.array([0.0, 0.0, 0.0, 0.0])

    dead_zone = np.array([0.1, 2, 0.05, 0.1])
    Kp = np.ones(3)
    Kd = np.ones(3)

    st = time.time()
    ct = time.time()
    while ct - st < 2:
        ct = time.time()
        line = ser.readline()

    while True:
        line = ser.readline()
        data = line.decode("utf-8")
        data_ls = data.split(",")

        fx = int(data_ls[0])/100.0
        fy = int(data_ls[1])/100.0
        fz = int(data_ls[2])/100.0

        tx = int(data_ls[3])/1000.0
        ty = int(data_ls[4])/1000.0
        tz = int(data_ls[5])/1000.0

        for j in range(len(traj_tau)):
            line = ser.readline()
            data = line.decode("utf-8") # in sequence pos, vel, torque
            data_ls = data.split(",")

            for i in range(6, 10):
                axis_d = data_ls[i].split(" ")
                raw_q[i-6] = enc2deg(int(axis_d[0]) - encOffset[i - 6], i - 6)
                raw_q_dot[i-6] = rpm2rad(int(axis_d[1]), i - 6)
                raw_tau[i-6] = float(axis_d[2])

            # cur_q_deg = enc2deg(raw_q[1] - encOffset[1], 1)
            q_sim = opt.robot2SimRad(raw_q * np.array([1, 1, -1, 1]))
            dq_sim = raw_q_dot * np.array([1, -1, -1, 1])
            
            pin.framesForwardKinematics(opt.model, robotData, traj_pos[j])
            Jtool = pin.computeFrameJacobian(opt.model, robotData, traj_pos[j], IDX_TOOL)

            ee_pos_plan = robotData.oMf[IDX_TOOL].translation
            ee_vel_plan = Jtool @ traj_vel[j]
            ee_vel_plan = ee_vel_plan[0:3]

            # pin.forwardKinematics(opt.model, robotData, q_sim)
            pin.framesForwardKinematics(opt.model, robotData, q_sim)
            Jtool = pin.computeFrameJacobian(opt.model, robotData, q_sim, IDX_TOOL)

            ee_pos = robotData.oMf[IDX_TOOL].translation
            ee_vel = Jtool @ q_sim
            ee_vel = ee_vel[0:3]



            # prev_tau = pin.nle(opt.model, robotData, q_sim, dq_sim) + params[-4:]
            # prev_tau *= np.array([1, -0.9, -1, 1.07])  # [1, -1.15, -1, 1.07]

            f_e = Kp * (ee_pos_plan - ee_pos) + Kd * (ee_vel_plan - ee_vel)
            f_e = np.hstack((f_e, np.zeros(3)))

            # wrench = np.array([-fy, fx, fz, -ty, tx, tz]) - wrenchOffset

            w_tau = (Jtool.T @ f_e) * np.array([1.0, -1.0, 1.0, -1.0])
            # prev_tau[1] = b[1]

            # prev_tau[0] = 1.6 + (q_sim[0] - (-np.pi / 2)) / (135 / 180 * np.pi) * 0.4

            # cur_tau = np.array(raw_tau)
            cur_tau = traj_tau[j] #+ w_tau
            print(cur_tau)
            print(w_tau)
            # cur_tau[3] = w_tau[3]
            # diff = cur_tau - prev_tau
            # diff[3] = w_tau[3]
            
            command = '#' + START + TORQ 

            pred_curr = np.array([0, 0, 0, 0])
            prev_curr = np.array([0, 0, 0, 0])

            #axis 1, no torque
            cur_tau[0] = torq2curr(cur_tau[0], 0)
            cur_tau[0] = max(min(cur_tau[0], mt[0]), 0)
            command += to_char(round(cur_tau[0]))



            #axis 2, gravity compensation
            cur_tau[1] = torq2curr(cur_tau[1], 1)
            cur_tau[1] = max(min(cur_tau[1], mt[1]), 0)
            command += to_char(round(cur_tau[1]))

            spo_torq = pred_curr[1]

            #axis 3, torq sensor amplify
            cur_tau[2] = torq2curr(cur_tau[2], 2)
            cur_tau[2] = max(min(cur_tau[2], mt[2]), -mt[2])
            command += to_char(round(cur_tau[3]))

            cur_tau[3] = torq2curr(cur_tau[3], 3) 
            cur_tau[3] = max(min(cur_tau[3], mt[3]), -mt[3])
            command += to_char(round(cur_tau[3]))

            print(command)
            ser.write(command.encode())



except KeyboardInterrupt:
    time.sleep(0.1)
    print("power off")
    print(spo_torq)
    while spo_torq > 5:
        line = ser.readline()
        command = '#' + START + TORQ + to_char(0) + to_char(round(spo_torq)) + to_char(0) + to_char(0)
        spo_torq = spo_torq - 1
        print(command)
        ser.write(command.encode())

    line = ser.readline()
    command = '#' + HALT 
    print(command)
    ser.write(command.encode())
    ser.close()
    print("Serial Port Closed")
    raise

