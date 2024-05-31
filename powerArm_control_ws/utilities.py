import numpy as np
import serial
from scipy.spatial import KDTree

CPR = 3600
GR = [26.3, 26.3, 42.6*3, 42.6]

raw_q = np.array([0.0, 0.0, 0.0, 0.0])
raw_tau = np.array([0.0, 0.0, 0.0, 0.0])
raw_q_dot = np.array([0.0, 0.0, 0.0, 0.0])
raw_tau = np.array([0.0, 0.0, 0.0, 0.0])

encOffset = [157, -270, 11000, -9300]  ## 4. 115020 -- 105720
tauOffset = [1.5, 0, 0,0.16] 

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

def read_data(ser):
    '''
    Raw data from sensors. Wrench in Sim coordinate.
    '''
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
        raw_tau[i - 6] = axis_d[2]

    wrench = np.array([-fy, fx, fz, -ty, tx, tz])

    return wrench, raw_q, raw_q_dot, raw_tau


def build_KDTree(data):
    tree = KDTree(data)

    return tree