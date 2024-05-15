import numpy as np

CPR = 3600
GR = [26.3, 26.3, 42.6*3, 42.6]

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

def load_data(file_name):
    data = np.load(file_name)
    q_pos = data['q_pos']
    q_vel = data['q_vel']
    q_tau = data['q_tau']
    wrenchOffset = data['wrenchOffset']
