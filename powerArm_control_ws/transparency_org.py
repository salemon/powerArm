import pinocchio as pin
import numpy as np
import math
from sys import argv
from os.path import dirname, join, abspath
import serial
import numpy as np
import time
import queue

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

# massOffset = [, , , ]
# -90 deg torque[, , -1.23], -45 [, , -1.65,] -120 [, , -1]
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

serial_port = '/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_002900343532511431333430-if02'
ser = serial.Serial(serial_port, baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS, timeout=0.5)
print("connected to: " + ser.portstr)
junk = ser.readall()
print(junk)
ser.close()
print("Junk clear")

ser = serial.Serial(serial_port, baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS)
print("connected to: " + ser.portstr)

test_posj = np.array([[90, 0, -90, 0], [90, 45, -45, 45], [60, 90, -90, 90], [90, 90, -90, 0]])
trans_posj = (test_posj + np.array([-90, -90, 90, 0])) * np.array([1, -1, -1, 1])
# print(trans_posj)


test_posj = np.array([[90, 90, -90, 0]])
trans_posj = (test_posj + np.array([-90, -90, 90, 0])) * np.array([1, -1, -1, 1])
print(trans_posj)

# i = 0
# command = '#' + START + POS + to_char(deg2enc(test_posj[i][0], 0) - encOffset[0]) \
#                             + to_char(deg2enc(test_posj[i][1], 1) - encOffset[1]) \
#                             + to_char(deg2enc(test_posj[i][2], 2) - encOffset[2]) \
#                             + to_char(deg2enc(test_posj[i][3], 3) - encOffset[3]) + "\r"
# print(command)

try:
    i = 0
    command = '#' + START + POS + to_char(deg2enc(test_posj[i][0], 0) + encOffset[0]) \
                                + to_char(deg2enc(test_posj[i][1], 1) + encOffset[1]) \
                                + to_char(deg2enc(test_posj[i][2], 2) + encOffset[2]) \
                                + to_char(deg2enc(test_posj[i][3], 3) + encOffset[3]) + "\r"
    print(command)
    ser.write(command.encode())

    st = time.time()
    ct = time.time()
    q = queue.Queue()
    a = 6 # motor rpm
    v = 200 # motor v rpm

    while ct - st < 5:
        ct = time.time()
        line = ser.readline()
        data = line.decode("utf-8")
        data_ls = data.split(",")

        raw_tau = []

        for i in range(4):
            axis_d = data_ls[i].split(" ")
            # print(axis_d)
            raw_tau.append(float(axis_d[2]))
        print(raw_tau)
        q.put(np.array(raw_tau))

        if q.qsize() > 3:
            q.get()

        q_sum = np.array([0.0, 0.0, 0.0, 0.0])
        for item in list(q.queue):
            q_sum += item

        prev_tau = q_sum / 3
        # prev_v = 
    
    print(prev_tau)
    print("Calibration Done")

    prev_v = np.array([0, 0, 0, 0])
    raw_q = np.array([0, 0, 0, 0])

    while True:
        st = time.time()
        line = ser.readline()
        data = line.decode("utf-8")
        data_ls = data.split(",")

        raw_tau.clear()
        for i in range(4):
            axis_d = data_ls[i].split(" ")
            raw_q[i] = axis_d[0]
            raw_tau.append(float(axis_d[2]))

        q.put(np.array(raw_tau))

        if q.qsize() > 3:
            q.get()

        q_sum = np.array([0.0, 0.0, 0.0, 0.0])
        for item in list(q.queue):
            q_sum += item

        cur_filter = q_sum / 3

        cur_q_deg = enc2deg(raw_q[1] - encOffset[1], 1)
        prev_tau[0] = 1.68
        prev_tau[1] = gravity(cur_q_deg)

        cur_tau = np.array(raw_tau)
        # cur_tau = cur_filter
        diff = cur_tau - prev_tau
        # prev = cur
        
        # print(diff)
        print("cur_deg: ", cur_q_deg)
        print("prev_tau: ", prev_tau)
        print("measured_tau: ", cur_tau)
        command = '#' + START + VEL #+ '+000000'
        
        pred_v = np.array([0, 0, 0, 0])
        # for i in range(4):
        i = 0
        pred_v[i] = rad2rpm(-(np.sign(diff[i]) * (abs(diff[i]) - 1)) if abs(diff[i]) > 1 else 0, i) / 2
        pred_v[i] = round(prev_v[i] + max(min(pred_v[i] - prev_v[i], a), -a))
        pred_v[i] = max(min(pred_v[0], v), -v)
        command += to_char(round(pred_v[i]))

        i = 1
        # command += to_char(deg2enc((-(np.sign(diff[1]) * abs(diff[1])) * 4 if abs(diff[1]) > 0.7 else 0) / np.pi * 180, 0))

        pred_v[i] = rad2rpm(-(np.sign(diff[i]) * (abs(diff[i]) - 2)) if abs(diff[i]) > 2 else 0, i) / 2
        pred_v[i] = round(prev_v[i] + max(min(pred_v[i] - prev_v[i], a), -a))
        pred_v[i] = max(min(pred_v[i], v), -v)

        prev_v = pred_v
        
        command += to_char(round(pred_v[i]))
        # command += to_char(round(0 / 23.5 * 100))

        command += '+000000' * 2 + '/r'
        print(command)
        ser.write(command.encode())
        # ct = time.time()
        # print(ct - st)
    # command = '#' + HALT

    # print("Test, finished")

    command = '#' + HALT
    ser.write(command.encode())

except KeyboardInterrupt:
    time.sleep(0.1)
    command = '#' + HALT
    print(command)
    ser.write(command.encode())
    ser.close()
    print("Serial Port Closed")
    raise


pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "powerArm_control_ws")
 
urdf_filename = pinocchio_model_dir + '/powerarm_urdf.urdf' if len(argv)<2 else argv[1]
model = pin.buildModelFromUrdf(urdf_filename)

data = model.createData()
 
qv = np.array([0, 0, 0, 0]) / 180 * np.pi


model.inertias[3].mass -= 0.5
# model.inertias[3].lever[0] -= 0.1
# print(model.inertias[3].lever)
model.inertias[4].mass -= 0.05
# print(*model.inertias)

# q = trans_posj[i] / 180 * np.pi
# pin.forwardKinematics(model, data, q)
# pin.framesForwardKinematics(model, data, q)

# idx = model.getFrameId("ee_link")

# M = pin.crba(model, data, q)
# b = pin.nle(model, data, q, qv)

# print('q: %s' % q.T)
# print(b)

for i in range(len(trans_posj)):
    q = trans_posj[i] / 180 * np.pi
    pin.forwardKinematics(model, data, q)
    pin.framesForwardKinematics(model, data, q)

    # idx = model.getFrameId("ee_link")

    M = pin.crba(model, data, q)
    b = pin.nle(model, data, q, qv)

    print('q: %s' % q.T)
    print(b)

# print(model.inertias[3])

# for i in range(model.nframes):
#     frame = model.frames[i]
#     print(frame.name)
#     print(model.names)
#     link_name = model.names[frame.name]

#     # Get the inertia properties from Pinocchio
#     if frame.parent > 0:
#         inertia = model.inertias[frame.inertia]
#         mass = inertia.mass
#         com = inertia.com
#         inertia_matrix = inertia.inertia

#         # Print inertia properties
#         print(f"Link: {link_name}")
#         print(f"  Mass: {mass}")
#         print(f"  Center of Mass (COM): {com}")
#         print(f"  Inertia Matrix:\n{inertia_matrix}")


# C = pin.computeCoriolisMatrix(model, data, q, qv)
# G = pin.computeGeneralizedGravity(model, data, q)

# print(C)
# print(G)


# test_data = ([['23670 0 1.47', '0 0 3.05', '-115021 0 -0.58', '0 0 -1.22'],
#               ['23670 0 1.55', '11836 0 25.77', '-57509 0 -0.72', '19170 0 -0.41'],
#               ['15780 0 1.42', '23671 0 34.04', '-115020 0 -1.12', '38340 0 0.27'],
#               ['23670 0 1.66', '23670 0 34.86', '-115020 0 0.28', '0 0 0.31']])

# for d in test_data:
#     cur_q = []
#     cur_tau = []
#     for j in range(len(d)):
#         dj = d[j].split(' ')
#         cur_q.append(round(enc2deg(float(dj[0]), j)))
#         cur_tau.append(dj[2])
#     print("q: ", cur_q)
#     print('tau: ', cur_tau)
    
# []