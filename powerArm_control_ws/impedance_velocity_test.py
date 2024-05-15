import pinocchio as pin
import numpy as np
import math
from sys import argv
from os.path import dirname, join, abspath
import serial
import numpy as np
import time
import queue
from optimization import dynamic_optimization


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
tauOffset = [1.5, 0, 0,0.16] 

initial_pos = [50, 90, -82, 40]

def to_char(value):
    chr_val = str(abs(value))
    num_zero = 6 - len(chr_val)
    zero_ph = num_zero * "0"

    if value < 0:
        return "-" + zero_ph + chr_val
    else:
        return "+" + zero_ph + chr_val

def rpm2rad(value, motor_num):
    return value * np.pi / 180 / GR[motor_num] * 360 / 60
    
def rad2rpm(value, motor_num):
    return value / np.pi * 180 * GR[motor_num] / 360 * 60

def enc2deg(value, motor_num):
    return value / CPR / GR[motor_num] * 360

def deg2enc(value, motor_num):
    return round(value * CPR * GR[motor_num] / 360)

def gravity(q):
    p1 = 1.181e-7
    p2 = -4.115e-5
    p3 = 0.0005695
    p4 = 0.5493
    p5 = 3.416

    tau = p1 * q**4 + p2 * q**3 + p3 * q**2 + p4 * q + p5

    return tau

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
    traj_vel.append([float(temp[i]) for i in range(4, 8)])

file.close()

opt = dynamic_optimization()
with open('optimized_params_2.npy', 'rb') as f:
    params = np.load(f)

traj_pos = np.array(traj_pos) # rad 
traj_vel = np.array(traj_vel) # rad / s

IDX_TOOL = opt.model.getFrameId('ee_link')

opt.setParameters(params)
robotData = opt.model.createData()

# with open('wrench_offset.npy', 'rb') as f:
#     wrenchOffset = np.load(f)


serial_port = '/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_002900343532511431333430-if02'
ser = serial.Serial(serial_port, baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS, timeout=0.5)
print("connected to: " + ser.portstr)
junk = ser.readall()
print(junk)
ser.close()
print("Junk clear")

ser = serial.Serial(serial_port, baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS)
print("connected to: " + ser.portstr)

try:
    command = '#' + START + POS + to_char(deg2enc(initial_pos[0], 0) + encOffset[0]) \
                                + to_char(deg2enc(initial_pos[1], 1) + encOffset[1]) \
                                + to_char(deg2enc(initial_pos[2], 2) + encOffset[2]) \
                                + to_char(deg2enc(initial_pos[3], 3) + encOffset[3]) + "\r"
    print(command)
    ser.write(command.encode())

    st = time.time()
    ct = time.time()
    q = queue.Queue()
    a = 6 # motor rpm
    v = 200 # motor v rpm
    raw_q = np.array([0.0, 0.0, 0.0, 0.0])
    raw_tau = np.array([0.0, 0.0, 0.0, 0.0])
    raw_q_dot = np.array([0.0, 0.0, 0.0, 0.0])
    raw_tau = np.array([0.0, 0.0, 0.0, 0.0])

    while ct - st < 3:
        ct = time.time()
        line = ser.readline()
        data = line.decode("utf-8")
        data_ls = data.split(",")


        fx = int(data_ls[0])/100.0
        fy = int(data_ls[1])/100.0
        fz = int(data_ls[2])/100.0

        tx = int(data_ls[3])/1000.0
        ty = int(data_ls[4])/1000.0
        tz = int(data_ls[5])/1000.0

        for i in range(6, 10):
            axis_d = data_ls[i].split(" ")
            raw_q[i - 6] = enc2deg(int(axis_d[0]) - encOffset[i - 6], i - 6)
            raw_tau[i - 6] = axis_d[2]
            
        # print(raw_tau)
        q.put(np.array(raw_tau))

        if q.qsize() > 3:
            q.get()

        q_sum = np.array([0.0, 0.0, 0.0, 0.0])
        for item in list(q.queue):
            q_sum += item

        prev_tau = q_sum / 3
        # prev_v = 
    wrenchOffset = np.array([-fy, fx, fz, -ty, tx, tz])
    prev_q = raw_q
    print(raw_q)
    prev_q = opt.robot2SimRad(raw_q * np.array([1, 1, -1, 1]))
    print(prev_q)
    print(prev_q / np.pi * 180)

    print("Calibration Done")

    Kp = 50 * np.ones(3)
    Kd = 0 * np.ones(3)

    k = 0
    n = 2
    vel_const = np.array([0.05, 0.1, 0.1, 0.05])
    # wrenchOffset[2] = -5
    
    # while k < n:
    prev_v = np.zeros(4)

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

        for i in range(6, 10):
            axis_d = data_ls[i].split(" ")
            raw_q[i - 6] = enc2deg(int(axis_d[0]) - encOffset[i - 6], i - 6)
            raw_q_dot[i - 6] = rpm2rad(int(axis_d[1]), i - 6)
            raw_tau = (float(axis_d[2]))

        q_act = opt.robot2SimRad(raw_q * np.array([1, 1, -1, 1]))
        dq_act = raw_q_dot * np.array([1, -1, -1, 1])

        # print(q_act)

        pin.framesForwardKinematics(opt.model, robotData, prev_q)
        Jtool = np.copy(pin.computeFrameJacobian(opt.model, robotData, prev_q, IDX_TOOL))
        ee_pos_plan = np.copy(robotData.oMf[IDX_TOOL].translation)
        ee_vel_plan = np.zeros(3)
        

        pin.framesForwardKinematics(opt.model, robotData, q_act)
        Jtool = np.copy(pin.computeFrameJacobian(opt.model, robotData, q_act, IDX_TOOL))

        ee_pos = np.copy(robotData.oMf[IDX_TOOL].translation)
        ee_vel = Jtool @ dq_act
        ee_vel = ee_vel[0:3]

        f_e = Kp * (ee_pos_plan - ee_pos) + Kd * (ee_vel_plan - ee_vel)
        f_e = np.hstack((f_e, np.zeros(3)))
        f_e[0], f_e[1] = -f_e[1], f_e[0]
        print("f_e: ", f_e)

        print(Kp * (ee_pos_plan - ee_pos))
        print(Kd * (ee_vel_plan - ee_vel))
        # wrench = np.array([-fy, fx, fz, -ty, tx, tz]) - wrenchOffset[0]
        wrench = np.array([-fy, fx, fz, -ty, tx, tz]) - wrenchOffset
        wrench[3:] = 0
        print("wrench: ", wrench)

        f_e  = np.array([(np.sign(f_e[i]) * (abs(f_e[i]) - 0.05)) if abs(f_e[i]) > 0.05 else 0 for i in range(6)])
        wrench = np.array([(np.sign(wrench[i]) * (abs(wrench[i]) - 1)) if abs(wrench[i]) > 1 else 0 for i in range(6)])
        ee_force = f_e + wrench


        print("ee_force: ", ee_force)
        joints_tau = (Jtool.T @ ee_force)

        pred_v = joints_tau * np.array([1, -1, -1, 1])
        print("pred_v: ", pred_v)

        command = '#' + START + VEL 
            
        for i in range(4):
            if i == 1:
                temp_v = pred_v[i] * vel_const[i]
                temp_v = rad2rpm(temp_v, i)
            else:
                temp_v = pred_v[i] * vel_const[i]
                temp_v = prev_v[i] + min(max(rad2rpm(temp_v, i) - prev_v[i], -6), 6)

                prev_v[i] = temp_v

            command += to_char(round(temp_v))
        print("prev_v: ", prev_v)
        
        ser.write(command.encode())
        print("\n")



        # for j in range(len(traj_vel)):

        #     line = ser.readline()
        #     data = line.decode("utf-8")
        #     data_ls = data.split(",")

        #     fx = int(data_ls[0])/100.0
        #     fy = int(data_ls[1])/100.0
        #     fz = int(data_ls[2])/100.0

        #     tx = int(data_ls[3])/1000.0
        #     ty = int(data_ls[4])/1000.0
        #     tz = int(data_ls[5])/1000.0

        #     for i in range(6, 10):
        #         axis_d = data_ls[i].split(" ")
        #         raw_q[i - 6] = enc2deg(int(axis_d[0]) - encOffset[i - 6], i - 6)
        #         raw_q_dot[i - 6] = rpm2rad(int(axis_d[1]), i - 6)
        #         raw_tau = (float(axis_d[2]))


        #     pin.framesForwardKinematics(opt.model, robotData, traj_pos[j])
        #     Jtool = pin.computeFrameJacobian(opt.model, robotData, traj_pos[j], IDX_TOOL)

        #     ee_pos_plan = robotData.oMf[IDX_TOOL].translation
        #     ee_vel_plan = Jtool @ traj_vel[j]
        #     ee_vel_plan = ee_vel_plan[0:3]

        #     q_act = opt.robot2SimRad(raw_q * np.array([1, 1, -1, 1]))
        #     dq_act = raw_q_dot * np.array([1, -1, -1, 1])

        #     pin.framesForwardKinematics(opt.model, robotData, q_act)
        #     Jtool = pin.computeFrameJacobian(opt.model, robotData, q_act, IDX_TOOL)

        #     ee_pos = robotData.oMf[IDX_TOOL].translation
        #     ee_vel = Jtool @ dq_act
        #     ee_vel = ee_vel[0:3]

        #     f_e = Kp * (ee_pos_plan - ee_pos) + Kd * (ee_vel_plan - ee_vel)
        #     f_e = np.hstack((f_e, np.zeros(3)))

        #     wrench = np.array([-fy, fx, fz, -ty, tx, tz]) - wrenchOffset[j]
        #     wrench[3:] = 0
        #     # ee_force = (f_e/0.2).astype(int) * 0.2 + (wrench / 0.2).astype(int) * 0.2
        #     f_e  = np.array([(np.sign(f_e[i]) * (abs(f_e[i]) - 1)) if abs(f_e[i]) > 1 else 0 for i in range(6)])
        #     wrench = np.array([(np.sign(wrench[i]) * (abs(wrench[i]) - 1)) if abs(wrench[i]) > 1 else 0 for i in range(6)])

        #     ee_force = f_e + wrench

        #     print(f_e)
        #     print(wrench)

        #     print(ee_force)
        #     # check ee_force correct?

        #     # joints_tau = (Jtool.T @ ee_force) * np.array([1.0, -1.0, 1.0, -1.0])
        #     joints_tau = (Jtool.T @ ee_force)
        #     # check w_tau correct?
            
        #     pred_v = np.copy(traj_vel[j]) + joints_tau * vel_const
        #     # pred_v = traj_vel[j] + joints_tau * vel_const
        #     pred_v *= np.array([1, -1, -1, 1])

        #     command = '#' + START + VEL 
            
        #     for i in range(4):
        #         command += to_char(round(rad2rpm(pred_v[i], i)))

        #     print(command)
        #     ser.write(command.encode())
        # k += 1

    # command = '#' + HALT
    # ser.write(command.encode())
        


        # i = 0
        # pred_v[i] = rad2rpm(-(np.sign(diff[i]) * (abs(diff[i]) - 1)) if abs(diff[i]) > 1 else 0, i) / 2
        # pred_v[i] = round(prev_v[i] + max(min(pred_v[i] - prev_v[i], a), -a))
        # pred_v[i] = max(min(pred_v[0], v), -v)
        # command += to_char(round(pred_v[i]))

        # i = 1
        # # command += to_char(deg2enc((-(np.sign(diff[1]) * abs(diff[1])) * 4 if abs(diff[1]) > 0.7 else 0) / np.pi * 180, 0))

        # pred_v[i] = rad2rpm(-(np.sign(diff[i]) * (abs(diff[i]) - 2)) if abs(diff[i]) > 2 else 0, i) / 2
        # pred_v[i] = round(prev_v[i] + max(min(pred_v[i] - prev_v[i], a), -a))
        # pred_v[i] = max(min(pred_v[i], v), -v)

        # prev_v = pred_v
        
        # command += to_char(round(pred_v[i]))
        # # command += to_char(round(0 / 23.5 * 100))

        # command += '+000000' * 2 + '/r'
        # print(command)
        # ser.write(command.encode())
        # ct = time.time()
        # print(ct - st)
    # command = '#' + HALT

    # print("Test, finished")

except KeyboardInterrupt:
    time.sleep(0.1)
    command = '#' + HALT
    print(command)
    ser.write(command.encode())
    ser.close()
    print("Serial Port Closed")
    raise
