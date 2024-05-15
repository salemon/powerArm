import serial
import numpy as np
import time
from optimization import dynamic_optimization
import pinocchio as pin

STOP = '0'
START = '2'
HOME = '1'
HALT = '3'

TORQ = '0'
VEL = '1'
POS = '2'

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

file = open('circle_points.txt','r')
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
    q_vel_list.append([float(temp[i]) for i in range(4, 8)])
    traj_vel.append([round(rad2rpm(float(temp[i]), i % 4)) for i in range(4, 8)])

file.close()

opt = dynamic_optimization()

with open('optimized_params_2.npy', 'rb') as f:
    params = np.load(f)

IDX_TOOL = opt.model.getFrameId('ee_link')

opt.setParameters(params)
robotData = opt.model.createData()

print("pre-load finished")

serial_port = '/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_002900343532511431333430-if02'
ser = serial.Serial(serial_port, baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS, timeout=0.5)
print("connected to: " + ser.portstr)
junk = ser.readall()
print(junk)
ser.close()
print('junkabv')

#send commands
ser = serial.Serial(serial_port, baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS)
print("connected to: " + ser.portstr)
# command = '#' + START + VEL + to_char(0) + to_char(0) + to_char(0) + to_char(0) + "\r"
# print(command)
# ser.write(command.encode())


print("start")
try:
    command = '#' + START + POS + to_char(deg2enc(initial_pos[0], 0) + encOffset[0]) \
                                + to_char(deg2enc(initial_pos[1], 1) + encOffset[1]) \
                                + to_char(deg2enc(initial_pos[2], 2) + encOffset[2]) \
                                + to_char(deg2enc(initial_pos[3], 3) + encOffset[3]) + "\r"
    ser.write(command.encode())
    print(command)
    # input()

    st = time.time()
    ct = time.time()
    while ct - st < 5:
        ct = time.time()
        line = ser.readline()

    data = line.decode("utf-8")
    data_ls = data.split(",")
    print(data_ls)

    n = 1
    j = 0
    raw_q = np.array([0.0, 0.0, 0.0, 0.0])
    raw_tau = np.array([0.0, 0.0, 0.0, 0.0])
    raw_q_dot = np.array([0.0, 0.0, 0.0, 0.0])

    st = time.time()
    ct = time.time()
    while ct - st < 1:
        ct = time.time()
        command = '#' + START + VEL + to_char(0) + to_char(0) + to_char(0) +to_char(0)
        ser.write(command.encode())
        line = ser.readline()

    ser.write(command.encode())
    print(command)
    print("Homing done")

    while j < n:
        raw_tau_list = []
        wrench_list = []
        q_rad_list = []

        for i in range(len(traj_vel)):
            line = ser.readline()
            data = line.decode("utf-8") # in sequence pos, vel, torque
            data_ls = data.split(",")

            fx = int(data_ls[0])/100.0
            fy = int(data_ls[1])/100.0
            fz = int(data_ls[2])/100.0

            tx = int(data_ls[3])/1000.0
            ty = int(data_ls[4])/1000.0
            tz = int(data_ls[5])/1000.0

            wrench = np.array([-fy, fx, fz, -ty, tx, tz]) 
            temp_w = np.copy(wrench)
            wrench_list.append(temp_w)

            for k in range(6, 10):
                axis_d = data_ls[k].split(" ")
                raw_q[k - 6] = enc2deg(int(axis_d[0]) - encOffset[k - 6], k - 6)
                raw_tau[k - 6] = axis_d[2]
                raw_q_dot[k - 6] = int(axis_d[1])

            if j == 1:
                temp = np.copy(raw_tau)
                raw_tau_list.append(temp)

            q_sim = opt.robot2SimRad(raw_q * np.array([1, 1, -1, 1]))
            q_rad_list.append(q_sim)

            pin.forwardKinematics(opt.model, robotData, q_sim)
            pin.framesForwardKinematics(opt.model, robotData, q_sim)

            ee_sim = np.copy(robotData.oMf[IDX_TOOL].translation)

            q_traj = np.array(traj_pos[i])
            # print(q_traj)

            pin.forwardKinematics(opt.model, robotData, q_traj)
            pin.framesForwardKinematics(opt.model, robotData, q_traj)

            ee_traj = np.copy(robotData.oMf[IDX_TOOL].translation)
            
            print("velocity")
            print(traj_vel[i])
            print(q_vel_list[i])
            # print(raw_q_dot)
            raw_q_dot[1] /= 10
            raw_q_dot *= np.array([1, -1, -1, 1])
            print(raw_q_dot)
            print("--------------------------")
            # print(i)
            print(q_sim)
            print(q_traj)
            # print(ee_sim)
            # print(ee_traj)
            command = '#' + START + VEL + to_char(traj_vel[i][0]) + to_char(-traj_vel[i][1]) + to_char(-traj_vel[i][2]) + to_char(traj_vel[i][3]) + "\r"
            print(command)

            ser.write(command.encode())

        j += 1

    # with open('circle_tau.npy', 'wb') as f:
    #     np.save(f, np.array(raw_tau_list))

    # with open('wrench_offset.npy', 'wb') as f:
    #     np.save(f, np.array(wrench_list))

    np.savez('circle_traj.npz', q_pos=np.array(q_rad_list), q_vel=np.array(q_vel_list), q_tau=np.array(raw_tau_list), wrenchOffset=np.array(wrench_list))



    line = ser.readline()
    command = '#' + HALT
    ser.write(command.encode())
    time.sleep(0.1)
    command = '#' + STOP
    print(command)

except KeyboardInterrupt:
    command = '#' + HALT
    ser.write(command.encode())
    time.sleep(0.1)
    command = '#' + STOP
    print(command)
    ser.write(command.encode())
    ser.close()
    print("Serial Port Closed")
    raise

