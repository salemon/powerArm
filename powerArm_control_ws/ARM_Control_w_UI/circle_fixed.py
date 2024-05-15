import serial
import numpy as np
import time
import pygame
#------------------------------------py game stuff---------------------------------------
# initializing the constructor
pygame.init()
pygame.display.set_caption('FixTraj Mode')
# screen resolution
res = (1300, 696)
# opens up a window
screen = pygame.display.set_mode(res)

#-----------------load assets---------------------
programIcon = pygame.image.load('./assets/icon.png').convert_alpha()
pygame.display.set_icon(programIcon)

#load background
background_reset = pygame.image.load('./assets/traj/traj_reset.png').convert_alpha()
background_start = pygame.image.load('./assets/traj/traj_start.png').convert_alpha()
background = pygame.image.load('./assets/traj/traj.png').convert_alpha()
#font
xy_font = pygame.font.SysFont('Corbel', 35, bold=True )
min_max_font = pygame.font.SysFont('Corbel', 25, bold=False )
screen.blit(background_reset, (0, 0))
pygame.display.update()
#------------------------------------------------------------------------------

STOP = '0'
START = '2'
HOME = '1'
HALT = '3'

TORQ = '0'
VEL = '1'
POS = '2'

CPR = 3600
GR = [26.3, 26.3, 42.6*3, 42.6]

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

file = open('circle_points.txt','r')
traj_pos = []
traj_vel = []

content = file.readline()

while True:
    content = file.readline()
    if not content or len(content) < 9:
        break

    temp = content.split(",")
    traj_pos.append([round(rad2rpm(float(temp[i]), i % 4)) for i in range(4)])
    traj_vel.append([round(rad2rpm(float(temp[i]), i % 4)) for i in range(4, 8)])

file.close()

# serial_port = '/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_002900343532511431333430-if02'
ser = serial.Serial(sport, baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS, timeout=0.5)
print("connected to: " + ser.portstr)
junk = ser.readall()
print(junk)
ser.close()
print('junkabv')

#send commands
ser = serial.Serial(sport, baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS)
print("connected to: " + ser.portstr)
command = '#' + START + VEL + to_char(0) + to_char(0) + to_char(0) + to_char(0) 
print(command)
ser.write(command.encode())
homing_speed = [60, 60, 300, 60]
try:
    for i in range(4):
        line = ser.readline()
        data = line.decode("utf-8")
        data_ls = data.split(",")
        cur_enc = data_ls[i+6].split(" ")
        print(data_ls)
        print(cur_enc)

        while enc2deg(float(cur_enc[0]), i) - initial_pos[i] > 0.1:
            command = '#' + START + VEL + i * to_char(0) + to_char(-homing_speed[i]) + (3 - i) * to_char(0) 
            ser.write(command.encode())

            line = ser.readline()
            data = line.decode("utf-8")
            data_ls = data.split(",")
            cur_enc = data_ls[i+6].split(" ")
            print(cur_enc[0], enc2deg(int(cur_enc[0]), i), "11111")
        # print(enc2deg(int(cur_enc[0]), i))
        while enc2deg(float(cur_enc[0]), i) - initial_pos[i] < -0.1:
            command = '#' + START + VEL + i * to_char(0) + to_char(homing_speed[i]) + (3 - i) * to_char(0) 
            ser.write(command.encode())

            line = ser.readline()
            data = line.decode("utf-8")
            data_ls = data.split(",")
            cur_enc = data_ls[i+6].split(" ")
            print(cur_enc[0], enc2deg(int(cur_enc[0]), i), "22222")
    
    line = ser.readline()
    data = line.decode("utf-8") # in sequence pos, vel, torque
    print(data)
    command = '#' + START + VEL + to_char(0) + to_char(0) + to_char(0) + to_char(0)
    ser.write(command.encode())
    
    done = False
    screen.blit(background_start, (0, 0))
    
    while done == False:
        line = ser.readline()
        print(line)
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    done = True
        pygame.display.update()


    # ser = serial.Serial(serial_port, baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS, timeout=0.5)
    # print("connected to: " + ser.portstr)
    # junk = ser.readall()
    # print(junk)
    # ser.close()
    # print('junkabv')

    # #send commands
    # ser = serial.Serial(serial_port, baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS)
    # print("connected to: " + ser.portstr)
    # command = '#' + START + VEL + to_char(0) + to_char(0) + to_char(0) + to_char(0)
    # print(command)
    # ser.write(command.encode())
    # print("initialization finished")
    
    n = 5
    j = 0

    done = False
    screen.blit(background, (0, 0))

    while j < n and done == False:
        for i in range(len(traj_vel)):
            line = ser.readline()
            data = line.decode("utf-8") # in sequence pos, vel, torque
            print(data)
            command = '#' + START + VEL + to_char(traj_vel[i][0]) + to_char(-traj_vel[i][1]) + to_char(-traj_vel[i][2]) + to_char(traj_vel[i][3]) 
            print(command)
            print(i + 1)
            ser.write(command.encode())
            print("write success")
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True
            pygame.display.update()
        j += 1
        

    line = ser.readline()
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