import pinocchio as pin
import numpy as np
import math
import serial
import numpy as np
import time
import queue
from optimization import dynamic_optimization
import utilities
# from scipy.spatial import KDTree


TOP = '0'
START = '2'
HOME = '1'
HALT = '3'

VEL = '1'
POS = '2'
TORQ = '3'

initial_pos = [50, 90, -82, 40]

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

IDX_TOOL = opt.model.getFrameId('ee_link')

opt.setParameters(params)
robotData = opt.model.createData()

ee_pos = []

for i in range(q_pos.shape[0]):
    pin.framesForwardKinematics(opt.model, robotData, q_pos[i])
    ee_pos.append(np.copy(robotData.oMf[IDX_TOOL].translation))

ee_pos = np.array(ee_pos)

tree = utilities.build_KDTree(ee_pos)

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
    command = '#' + START + POS + utilities.to_char(utilities.deg2enc(initial_pos[0], 0) + utilities.encOffset[0]) \
                                + utilities.to_char(utilities.deg2enc(initial_pos[1], 1) + utilities.encOffset[1]) \
                                + utilities.to_char(utilities.deg2enc(initial_pos[2], 2) + utilities.encOffset[2]) \
                                + utilities.to_char(utilities.deg2enc(initial_pos[3], 3) + utilities.encOffset[3]) + "\r"
    print(command)
    ser.write(command.encode())

    st = time.time()
    ct = time.time()
    q = queue.Queue()
    a = np.array([6, 6, 20, 6]) # motor rpm
    v = 200 # motor v rpm

    while ct - st < 5:
        ct = time.time()
        line = ser.readline()
    
    st = time.time()
    ct = time.time()
    
    while ct - st < 1:
        ct = time.time()
        command = '#' + START + VEL + utilities.to_char(0) + utilities.to_char(0) + utilities.to_char(0) + utilities.to_char(0)
        ser.write(command.encode())
        line = ser.readline()

    print("Calibration Done")

    Kp = 80 * np.ones(3)
    Kd = 0 * np.ones(3)

    k = 0
    n = 3
    vel_const = np.array([0.05, 0.1, 0.05, 0.05]) * 0
    prev_v = np.zeros(4)

    while True:
        
        wrench, raw_q, raw_q_dot, raw_tau = utilities.read_data(ser)

        q_cur = opt.robot2SimRad(raw_q * np.array([1, 1, -1, 1]))

        pin.framesForwardKinematics(opt.model, robotData, q_cur)
        Jtool = np.copy(pin.computeFrameJacobian(opt.model, robotData, q_cur, IDX_TOOL))
        pos_cur = np.copy(robotData.oMf[IDX_TOOL].translation)

        _, cur_idx = tree.query(pos_cur)
        print(cur_idx)

        ee_traj = ee_pos[cur_idx]
        
        idx_left = cur_idx - 1
        idx_right = (cur_idx + 1) % q_pos.shape[0]

        vec_left = ee_pos[idx_left] - pos_cur
        vec_right = ee_pos[idx_right] - pos_cur

        vec_left[0], vec_left[1] = -vec_left[1], vec_left[0]
        vec_right[0], vec_right[1] = -vec_right[1], vec_right[0]
        
        wrench -= wrenchOffset[cur_idx]
        wrench[3:] = 0
        
        print("wrench: ", wrench)

        f_e = Kp * (ee_traj - pos_cur) #+ Kd * (ee_vel_plan - ee_vel)
        f_e = np.hstack((f_e, np.zeros(3)))
        f_e[0], f_e[1] = -f_e[1], f_e[0]
        print("f_e: ", f_e)

        # print(Kp * (ee_pos_plan - ee_pos))
        # print(Kd * (ee_vel_plan - ee_vel))


        f_e  = np.array([(np.sign(f_e[i]) * (abs(f_e[i]) - 0.05)) if abs(f_e[i]) > 0.05 else 0 for i in range(6)])
        wrench = np.array([(np.sign(wrench[i]) * (abs(wrench[i]) - 1)) if abs(wrench[i]) > 1 else 0 for i in range(6)])
        ee_force = f_e + wrench

        print("ee_force: ", ee_force)
        joints_tau = (Jtool.T @ ee_force)

        # pred_v = joints_tau * np.array([1, -1, -1, 1]) * vel_const
        pred_v = np.zeros(4)
        print("pred_v_exforce: ", pred_v)
        
        if wrench[:3] @ vec_left > wrench[:3] @ vec_right:
            idx_nxt = idx_left
        else:
            idx_nxt = idx_right

        pred_v += (q_pos[idx_nxt] - q_pos[cur_idx]) / 0.05 * np.array([1, -1, -1, 1])
        print("pred_v: ", pred_v)


        command = '#' + START + VEL 
            
        for i in range(4):
            temp_v = pred_v[i]
            temp_v = prev_v[i] + min(max(utilities.rad2rpm(temp_v, i) - prev_v[i], -a[i]), a[i])

            prev_v[i] = temp_v

            command += utilities.to_char(round(temp_v))
        print("prev_v: ", prev_v)
        
        ser.write(command.encode())
        print("\n")

    #     print("-----------------------------------------------------------------------\n")
    #     line = ser.readline()
    #     command = '#' + START + VEL + utilities.to_char(0) + utilities.to_char(0) + utilities.to_char(0) + utilities.to_char(0)
    #     ser.write(command.encode())
        
    #     k += 1

    # time.sleep(0.1)
    # command = '#' + HALT
    # print(command)
    # ser.write(command.encode())
    # ser.close()
    # print("Serial Port Closed")

except KeyboardInterrupt:
    time.sleep(0.1)
    command = '#' + HALT
    print(command)
    ser.write(command.encode())
    ser.close()
    print("Serial Port Closed")
    raise
