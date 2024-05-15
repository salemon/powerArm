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

data = np.load('circle_traj.npz')
q_pos = data['q_pos']
q_vel = data['q_vel']
q_tau = data['q_tau']
wrenchOffset = data['wrenchOffset']

print(q_pos / np.pi * 180)
print(q_vel)
print(wrenchOffset)
print("-------------------------")

opt = dynamic_optimization()
with open('optimized_params_2.npy', 'rb') as f:
    params = np.load(f)

# traj_pos = np.array(traj_pos) # rad 
# traj_vel = np.array(traj_vel) # rad / s

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
    a = np.array([6, 6, 50, 6]) # motor rpm
    v = 200 # motor v rpm
    raw_q = np.array([0.0, 0.0, 0.0, 0.0])
    raw_tau = np.array([0.0, 0.0, 0.0, 0.0])
    raw_q_dot = np.array([0.0, 0.0, 0.0, 0.0])
    raw_tau = np.array([0.0, 0.0, 0.0, 0.0])

    while ct - st < 5:
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
    
    st = time.time()
    ct = time.time()
    while ct - st < 1:
        ct = time.time()
        command = '#' + START + VEL + to_char(0) + to_char(0) + to_char(0) +to_char(0)
        ser.write(command.encode())
        line = ser.readline()
    # wrenchOffset = np.array([-fy, fx, fz, -ty, tx, tz])

    # prev_q = raw_q
    # prev_q = opt.robot2SimRad(raw_q * np.array([1, 1, -1, 1]))

    print("Calibration Done")

    Kp = 50 * np.ones(3)
    Kd = 0 * np.ones(3)

    k = 0
    n = 3
    vel_const = np.array([0.05, 0.1, 0.1, 0.05])
    # wrenchOffset[2] = -5
    prev_v = np.zeros(4)

    while k < n:
        for j in range(len(q_pos)):
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
            dq_act[1] /= 10
            # q_vel[j] *= np.array([1, -1, -1, 1])
            print(dq_act)
            print(q_vel[j])

            pin.framesForwardKinematics(opt.model, robotData, q_pos[j])
            Jtool = np.copy(pin.computeFrameJacobian(opt.model, robotData, q_pos[j], IDX_TOOL))
            ee_pos_plan = np.copy(robotData.oMf[IDX_TOOL].translation)
            ee_vel_plan = Jtool @ q_vel[j]
            ee_vel_plan = ee_vel_plan[:3]
            

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
            wrench = np.array([-fy, fx, fz, -ty, tx, tz]) - wrenchOffset[j]
            wrench[3:] = 0
            print("wrench: ", wrench)

            f_e  = np.array([(np.sign(f_e[i]) * (abs(f_e[i]) - 0.05)) if abs(f_e[i]) > 0.05 else 0 for i in range(6)])
            wrench = np.array([(np.sign(wrench[i]) * (abs(wrench[i]) - 1)) if abs(wrench[i]) > 1 else 0 for i in range(6)])
            ee_force = f_e + wrench


            print("ee_force: ", ee_force)
            joints_tau = (Jtool.T @ ee_force)

            pred_v = joints_tau * np.array([1, -1, -1, 1]) * vel_const
            print("pred_v_exforce: ", pred_v)
            pred_v += q_vel[j] * np.array([1, -1, -1, 1])
            print("pred_v: ", pred_v)


            command = '#' + START + VEL 
                
            for i in range(4):
                # if i == 1:
                #     temp_v = pred_v[i] * vel_const[i]
                #     temp_v = rad2rpm(temp_v, i)
                # else:
                temp_v = pred_v[i]
                temp_v = prev_v[i] + min(max(rad2rpm(temp_v, i) - prev_v[i], -a[i]), a[i])

                prev_v[i] = temp_v

                command += to_char(round(temp_v))
            print("prev_v: ", prev_v)
            
            ser.write(command.encode())
            print("\n")

        print("-----------------------------------------------------------------------\n")
        line = ser.readline()
        command = '#' + START + VEL + to_char(0) + to_char(0) + to_char(0) + to_char(0)
        ser.write(command.encode())
        
        k += 1

    # command = '#' + START + VEL + to_char(0) + to_char(0) + to_char(0) +to_char(0)
    time.sleep(0.1)
    command = '#' + HALT
    print(command)
    ser.write(command.encode())
    ser.close()
    print("Serial Port Closed")

except KeyboardInterrupt:
    time.sleep(0.1)
    command = '#' + HALT
    print(command)
    ser.write(command.encode())
    ser.close()
    print("Serial Port Closed")
    raise
