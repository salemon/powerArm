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
TorqConst = 7

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

def torq2curr(value):
    return value * TorqConst

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


test_posj = np.array([[90, 5, -90, 0]])

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
    a = [0,50,50,50] # max motor torq acc
    mt = [0,260, 800,100] # motor torq max
    while ct - st < 5:
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
    
    print(prev_tau)
    print("Calibration Done")


    spo_torq = 0
    raw_q = np.array([0, 0, 0, 0])
    raw_q_dot = np.array([0, 0, 0, 0])
    dead_zone = np.array([0, 2, 0.05, 0.1])
    while True:
        line = ser.readline()
        data = line.decode("utf-8")
        data_ls = data.split(",")
        raw_tau.clear()

        fx = int(data_ls[0])/100.0
        fy = int(data_ls[1])/100.0
        fz = int(data_ls[2])/100.0

        tx = int(data_ls[3])/1000.0
        ty = int(data_ls[4])/1000.0
        tz = int(data_ls[5])/1000.0

        for i in range(6,10):
            axis_d = data_ls[i].split(" ")
            raw_q[i-6] = axis_d[0]
            raw_q_dot[i-6] = axis_d[1]
            raw_tau.append(float(axis_d[2]))

        cur_q_deg = enc2deg(raw_q[1] - encOffset[1], 1)
        prev_tau[1] = gravity(cur_q_deg)

        cur_tau = np.array(raw_tau)
        diff = cur_tau - prev_tau
        
        # print(diff)
        # print("cur_deg: ", cur_q_deg)
        # print("prev_tau: ", prev_tau)
        # print("measured_tau: ", cur_tau)
        #print(fx,fy, fz, tx, ty, tz)
        command = '#' + START + TORQ 
        
        #axis 1, no torque
        command += to_char(0)
        #axis 2, gravity compensation
        pred_curr = np.array([0, 0, 0, 0])
        prev_curr = np.array([0, 0, 0, 0])
        
        pred_curr[1] = torq2curr(-(np.sign(diff[1]) * (abs(diff[1]) - dead_zone[1])) if abs(diff[1]) > dead_zone[1] else 0)
        pred_curr[1] = round(torq2curr(prev_tau[1]) + max(min(pred_curr[1], a[1]), -a[1])) 
        pred_curr[1] = max(min(pred_curr[1], mt[1]), 0)
        command += to_char(round(pred_curr[1]))

        prev_curr = pred_curr
        spo_torq = pred_curr[1]

        #axis 3, torq sensor amplify
        j3_torq = tx - 0.735 #joystick value
        pred_curr[2] = torq2curr(-(np.sign(j3_torq) * (abs(j3_torq) - dead_zone[2])) if abs(j3_torq) > dead_zone[2] else 0)
        pred_curr[2] = pred_curr[2]*8 #amp factor
        print(pred_curr[2])
        pred_curr[2] = max(min(pred_curr[2], mt[2]), -mt[2])
        print(j3_torq)
        print(raw_q_dot[2])
        # print(round(pred_curr[2]))

        torque_feedforward = -600
        #torque feedforward
        if (abs(raw_q_dot[2]) < 50 and abs(pred_curr[2]) > 0):
            pred_curr[2] = pred_curr[2] + torque_feedforward*np.sign(j3_torq)

        print(round(pred_curr[2]))

        #add torque feed
        command += to_char(round(pred_curr[2]))

        #axis 4, torque sensor amplify
        j4_torq = fy-38.6#joystick value
        pred_curr[3] = torq2curr(-(np.sign(j4_torq) * (abs(j4_torq) - dead_zone[2])) if abs(j4_torq) > dead_zone[3] else 0)
        pred_curr[3] = pred_curr[3]*-1 #amp factor
        pred_curr[3] = max(min(pred_curr[3], mt[3]), -mt[3])
        #print(j4_torq)
        #print(round(pred_curr[3]))
        command += to_char(round(pred_curr[3]))
    
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