#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import pygame
import sys
import time
import math
import random
from pygame.locals import *

import serial
import numpy as np
import time
import queue
import pygame 

#------------------------------------py game stuff---------------------------------------
# initializing the constructor
pygame.init()
pygame.display.set_caption('Assessment Mode')
# screen resolution
res = (1300, 696)
# opens up a window
screen = pygame.display.set_mode(res)

#-----------------load assets---------------------
programIcon = pygame.image.load('./assets/icon.png').convert_alpha()
pygame.display.set_icon(programIcon)

#load background
background_reset = pygame.image.load('./assets/assessment/assessment_reset.png').convert_alpha()
background_start = pygame.image.load('./assets/assessment/assessment_start.png').convert_alpha()
background = pygame.image.load('./assets/assessment/assessment.png').convert_alpha()
#font
xy_font = pygame.font.SysFont('Corbel', 35, bold=True )
min_max_font = pygame.font.SysFont('Corbel', 25, bold=False )
screen.blit(background_reset, (0, 0))
pygame.display.update()
#------------------------------------------------------------------------------

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

# serial_port = '/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_002900343532511431333430-if02'
ser = serial.Serial(sport, baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS, timeout=0.5)
print("connected to: " + ser.portstr)
junk = ser.readall()
print(junk)
ser.close()
print("Junk clear")

ser = serial.Serial(sport, baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS)
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
    mt = [0,260, 400,100] # motor torq max
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
        pygame.display.update()
    
    print(prev_tau)
    print("Calibration Done")
    
    #---------------------------------------wait for wearing-----------------------------------
    done = False
    screen.blit(background_start, (0, 0))
    
    while done == False:
        line = ser.readline()
        
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    done = True
        pygame.display.update()
        

    spo_torq = 0
    raw_q = np.array([0, 0, 0, 0])
    raw_q_dot = np.array([0, 0, 0, 0])
    dead_zone = np.array([0, 2, 0.05, 0.1])

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

#---------------------------------------------------above are robot init-------------------------------------------------------

#size = (width, height) = (1300, 696)
size = (width,height) = (1024,600)
black = (0, 0, 0)
white = (255, 255, 255)
green = (0, 155, 0)
red = (155, 0, 0)
sky = (0, 0, 0)
# clock = pygame.time.Clock()
FPS = 21
maxspeed = 15

screen = pygame.display.set_mode(size)
#screen = pygame.display.set_mode(size,DOUBLEBUF | FULLSCREEN)



def cpumove(cpu, target):
    if target.rect.left < cpu.rect.left:
        cpu.trigger = 1
        cpu.speed = -2
    elif target.rect.left > cpu.rect.left:
        cpu.trigger = 1
        cpu.speed = 2
    if random.randrange(0, 30) == 1:
        cpu.fire = 1
    else:
        cpu.fire = 0


def bossmove(cpu, target):
    if target.rect.left < cpu.rect.left and cpu.spree == False:
        cpu.trigger = 1
        cpu.speed = -2
    elif target.rect.left > cpu.rect.left and cpu.spree == False:
        cpu.trigger = 1
        cpu.speed = 2

    if random.randrange(0, 3) == 1 and cpu.spree == False:
        cpu.bulletformation = 0
        cpu.bulletspeed = 20
        cpu.fire = 1
    else:
        cpu.fire = 0

    if cpu.spree == False and random.randrange(0, 250) == 71:
        cpu.spree = True
    else:
        pass


def load_image(
    name,
    sizex=-1,
    sizey=-1,
    colorkey=None,
    ):

    fullname = os.path.join('Sprites', name)
    image = pygame.image.load(fullname)
    image = image.convert()
    if colorkey is not None:
        if colorkey is -1:
            colorkey = image.get_at((0, 0))
        image.set_colorkey(colorkey, RLEACCEL)

    if sizex != -1 or sizey != -1:
        image = pygame.transform.scale(image, (sizex, sizey))

    return (image, image.get_rect())


def showhealthbar(
    health,
    barcolor,
    pos,
    unit,
    ):

    healthbar = pygame.Surface((health * unit, 10), pygame.SRCALPHA, 32)
    healthbar = healthbar.convert_alpha()
    pygame.draw.rect(screen, barcolor, pos)


def displaytext(
    text,
    fontsize,
    x,
    y,
    color,
    ):

    font = pygame.font.SysFont('sawasdee', fontsize, True)
    text = font.render(text, 1, color)
    textpos = text.get_rect(centerx=x, centery=y)
    screen.blit(text, textpos)


def moveplayer(Player):
    if Player.isautopilot == False:
        if Player.rect.left >= 0 and Player.rect.right <= width:
            if Player.trigger == 1:
                Player.movement[0] = Player.movement[0] + Player.speed
                if Player.movement[0] < -maxspeed:
                    Player.movement[0] = -maxspeed
                elif Player.movement[0] > maxspeed:
                    Player.movement[0] = maxspeed
            elif Player.movement[0] >= -maxspeed and Player.movement[0] \
                < 0 and Player.trigger == 2:
                Player.movement[0] += math.fabs(Player.movement[0] / 20)
                if Player.movement[0] > 0:
                    Player.movement[0] = 0
            elif Player.movement[0] <= maxspeed and Player.movement[0] \
                > 0 and Player.trigger == 2:
                Player.movement[0] -= math.fabs(Player.movement[0] / 20)
                if Player.movement[0] < 0:
                    Player.movement[0] = 0
    else:
        Player.autopilot()


def storyboard(wavecounter):
    if wavecounter >= 0 and wavecounter <= 700:  # enemy
        return 0
    elif wavecounter > 700 and wavecounter <= 1100:

                                                     # saucer

        return 1
    elif wavecounter > 1100 and wavecounter <= 1500:

                                                     # drone

        return 2
    elif wavecounter > 1500 and wavecounter <= 1800:

                                                     # station

        return 3
    elif wavecounter > 1800 and wavecounter <= 2300:

                                                     # drone

        return 4
    elif wavecounter > 2300 and wavecounter <= 2700:

                                                     # enemy and saucer

        return 5
    elif wavecounter > 2700 and wavecounter <= 2900:

                                                     # enemy

        return 6
    elif wavecounter > 2900 and wavecounter <= 3300:

                                                     # drone and saucer

        return 7
    elif wavecounter > 3300 and wavecounter <= 3600:

                                                     # saucer

        return 8
    elif wavecounter > 3600 and wavecounter <= 4000:

                                                     # enemy and drones

        return 9
    elif wavecounter > 4000 and wavecounter <= 4400:

                                                     # station

        return 10
    elif wavecounter > 4400:

                             # boss

        return 11


class stars:

    def __init__(self,radius,color,nofstars,speed=2):
        self.radius = radius
        self.color = color
        self.speed = speed
        self.nofstars = nofstars
        self.starpos = [[0 for j in range(2)] for i in range(self.nofstars)]
        for x in range(self.nofstars):
            self.starpos[x][0] = random.randrange(0, width)
            self.starpos[x][1] = random.randrange(0, height)

    def drawstars(self):
        for x in range(self.nofstars):
            #pygame.draw.rect(screen, color, [self.starpos[x][0],
            #                 self.starpos[x][1], 2, 2])
            pygame.draw.circle(screen,self.color,(self.starpos[x][0],self.starpos[x][1]),self.radius)
        self.movestars()

    def movestars(self):
        for x in range(self.nofstars):
            self.starpos[x][1] += self.speed
            if self.starpos[x][1] > height:
                self.starpos[x][1] = 0



class player(pygame.sprite.Sprite):

    def __init__(self):
        pygame.sprite.Sprite.__init__(self)
        (self.image, self.rect) = load_image('fighter1_scale.png', 72,
                72, -1)
        self.rect.top = size[1] - 100#500
        self.rect.left = size[0]/2#200

        self.speed = 0
        self.fire = 0
        self.movement = [0, 0]
        self.trigger = 0
        self.health = 100
        self.kills = 0
        self.score = 0
        self.shootdelay = 0
        self.isautopilot = False
        self.shot = False
        self.won = False

    def checkbounds(self):
        if self.rect.left < 0:
            self.rect.left = 0
            self.movement[0] = 0
            self.speed = 0
        if self.rect.right > width:
            self.rect.right = width
            self.movement[0] = 0
            self.speed = 0

    def update(self, x):
        # self.rect = self.rect.move(self.movement)
        self.rect.x = x

        self.shootdelay += 1
        if self.fire == 1 and self.shootdelay%3 == 1:
            self.shoot()

        if self.health > 200:
            self.health = 200

    def drawplayer(self):
        screen.blit(self.image, self.rect)

    def shoot(self):
        (x, y) = self.rect.center
        self.shot = bullet(x - 14, y, (0, 255, 0), 1)
        self.shot = bullet(x + 14, y, (0, 255, 0), 1)

    def autopilot(self):
        if self.rect.centerx < width / 2:
            self.movement[0] = 5
        else:
            self.movement[0] = -5
        if self.rect.centerx - width / 2 < 5 and self.rect.centerx \
            - width / 2 > -5:
            self.movement[0] = 0
            self.movement[1] = -10


class boss(pygame.sprite.Sprite):

    def __init__(self):
        pygame.sprite.Sprite.__init__(self)

        (self.image, self.rect) = load_image('boss.png', 200, 400, -1)#125, 250, -1)
        self.rect = self.image.get_rect()
        self.rect.top = 100
        self.rect.left = random.randrange(0, width - 72)

        self.speed = 0
        self.fire = 0
        self.movement = [0, 0]
        self.trigger = 0
        self.health = 600
        self.bulletformation = 0
        self.bulletspeed = 20
        self.spreecount = 0
        self.spree = False
        self.shot = False
        self.isautopilot = False
        self.reloadtime = 0

    def checkbounds(self):
        if self.rect.left < 0:
            self.rect.left = 0
            self.movement[0] = 0
            self.speed = 0
        if self.rect.right > width:
            self.rect.right = width
            self.movement[0] = 0
            self.speed = 0

    def update(self):
        self.checkbounds()
        moveplayer(self)

        self.rect = self.rect.move(self.movement)

        if self.fire == 1 and self.reloadtime == 0:
            self.shoot(self.bulletformation, self.bulletspeed)

        if self.reloadtime > 0:
            self.reloadtime -= 1

        if self.health <= 0:
            self.kill()

        if self.spree == True and self.spreecount <= 70:
            self.spreecount += 1
            if self.spreecount % 5 == 1:
                self.movement[0] = 0
                self.speed = 0
                self.shoot(1, 10)
            else:
                pass
        else:
            self.spree = False
            self.spreecount = 0

    def drawplayer(self):
        screen.blit(self.image, self.rect)

    def shoot(self, bulletformation=0, bulletspeed=20):
        (x, y) = self.rect.center
        if bulletformation == 0:
            self.shot = enemybullet(x, y + self.rect.height / 2, (255,
                                    0, 255), [0, 1], bulletspeed)
            self.shot = enemybullet(x - self.rect.width / 2 + 30, y
                                    - self.rect.height / 2 + 50, (255,
                                    0, 255), [0, 1], bulletspeed)
            self.shot = enemybullet(x + self.rect.width / 2 - 30, y
                                    - self.rect.height / 2 + 50, (255,
                                    0, 255), [0, 1], bulletspeed)
        elif bulletformation == 1:
            self.shot = enemybullet(x, y, (255, 0, 255), [1.5, 1],
                                    bulletspeed)
            self.shot = enemybullet(x, y, (255, 0, 255), [-1.5, 1],
                                    bulletspeed)
            self.shot = enemybullet(x, y, (255, 0, 255), [1.2, 1],
                                    bulletspeed)
            self.shot = enemybullet(x, y, (255, 0, 255), [-1.2, 1],
                                    bulletspeed)
            self.shot = enemybullet(x, y, (255, 0, 255), [0, 1],
                                    bulletspeed)
            self.shot = enemybullet(x, y, (255, 0, 255), [0.9, 1],
                                    bulletspeed)
            self.shot = enemybullet(x, y, (255, 0, 255), [-0.9, 1],
                                    bulletspeed)
            self.shot = enemybullet(x, y, (255, 0, 255), [0.6, 1],
                                    bulletspeed)
            self.shot = enemybullet(x, y, (255, 0, 255), [-0.6, 1],
                                    bulletspeed)
            self.shot = enemybullet(x, y, (255, 0, 255), [0.3, 1],
                                    bulletspeed)
            self.shot = enemybullet(x, y, (255, 0, 255), [-0.3, 1],
                                    bulletspeed)

        if random.randrange(0, 10) == 4:
            enemy(random.randrange(0, 4))
        if random.randrange(0, 50) == 41:
            enemysaucer(random.randrange(0, width - 50))
        if random.randrange(0, 200) == 121:
            enemydrone(random.randrange(0, width - 50))


class enemy(pygame.sprite.Sprite):

    def __init__(self, n=0):
        pygame.sprite.Sprite.__init__(self, self.containers)
        sheet = pygame.image.load('Sprites/enemy_sheet1.png')
        self.images = []

        rect = pygame.Rect((0, 0, 85, 92))
        image = pygame.Surface(rect.size)
        image.blit(sheet, (0, 0), rect)
        self.images.append(image)

        rect = pygame.Rect((86, 0, 71, 92))
        image = pygame.Surface(rect.size)
        image.blit(sheet, (0, 0), rect)
        self.images.append(image)

        rect = pygame.Rect((158, 0, 68, 92))
        image = pygame.Surface(rect.size)
        image.blit(sheet, (0, 0), rect)
        self.images.append(image)

        rect = pygame.Rect((227, 0, 65, 92))
        image = pygame.Surface(rect.size)
        image.blit(sheet, (0, 0), rect)
        self.images.append(image)

        self.image = self.images[n]
        self.image = self.image.convert()
        colorkey = -1
        colorkey = self.image.get_at((10, 10))
        self.image.set_colorkey(colorkey, RLEACCEL)

        self.image = pygame.transform.scale(self.image, (36, 36))
        self.rect = self.image.get_rect()

        self.image = pygame.transform.rotate(self.image, 180)
        self.rect.top = 0
        self.rect.left = random.randrange(0, width - 72)

        self.speed = 0
        self.fire = 0
        self.movement = [0, 0]
        self.trigger = 0
        self.health = 2
        self.isautopilot = False

        self.explosion_sound = \
            pygame.mixer.Sound('Sprites/explosion.wav')
        self.explosion_sound.set_volume(0.1)

        self.shot = False

    def checkbounds(self):
        if self.rect.left < 0:
            self.rect.left = 0
            self.movement[0] = 0
            self.speed = 0
        if self.rect.right > width:
            self.rect.right = width
            self.movement[0] = 0
            self.speed = 0

    def update(self):
        self.checkbounds()

        moveplayer(self)
        self.autopilot()
        self.rect = self.rect.move(self.movement)

        if self.fire == 1:
            self.shoot()

        if self.health <= 0:
            (x, y) = self.rect.center
            if pygame.mixer.get_init():
                self.explosion_sound.play(maxtime=1000)
            explosion(x, y)
            self.kill()

    def drawplayer(self):
        screen.blit(self.image, self.rect)

    def shoot(self):
        (x, y) = self.rect.center
        self.shot = enemybullet(x, y, (255, 255, 0), [0, 1], 12)

    def autopilot(self):
        if self.rect.top < height:
            self.movement[1] = 5
        else:
            self.kill()


class enemydrone(pygame.sprite.Sprite):

    def __init__(self, x):
        pygame.sprite.Sprite.__init__(self, self.containers)
        (self.image, self.rect) = load_image('enemy2_scale.png', 50,
                102, -1)
        self.rect.top = -self.rect.height
        self.rect.left = x

        self.speed = 0
        self.fire = 1
        self.movement = [0, 0]
        self.health = 20

        self.shot = False
        self.waitTime = 0
        self.explosion_sound = \
            pygame.mixer.Sound('Sprites/explosion.wav')
        self.explosion_sound.set_volume(0.1)

    def checkbounds(self):
        if self.rect.left < 0:
            self.rect.left = 0
            self.movement[0] = 0
            self.speed = 0
        if self.rect.right > width:
            self.rect.right = width
            self.movement[0] = 0
            self.speed = 0

    def update(self):
        self.checkbounds()
        self.autopilot()
        self.rect = self.rect.move(self.movement)

        if self.fire == 1 and self.waitTime % 10 == 1:
            self.shoot()

        if self.health <= 0:
            (x, y) = self.rect.center
            if pygame.mixer.get_init():
                self.explosion_sound.play(maxtime=1000)
            explosion(x, y,100)
            self.kill()

    def drawplayer(self):
        screen.blit(self.image, self.rect)

    def shoot(self):
        (x, y) = self.rect.center
        self.shot = enemybullet(x, y + self.rect.height / 2, (255, 0,
                                0), [0, 1], 10)
        self.shot = enemybullet(x, y + self.rect.height / 2, (255, 0,
                                0), [-0.5, 1], 10)
        self.shot = enemybullet(x, y + self.rect.height / 2, (255, 0,
                                0), [0.5, 1], 10)
        self.shot = enemybullet(x, y + self.rect.height / 2, (255, 0,
                                0), [-1, 1], 10)
        self.shot = enemybullet(x, y + self.rect.height / 2, (255, 0,
                                0), [1, 1], 10)

    def autopilot(self):
        if self.rect.top < height - 500:
            self.movement[1] = 3
        elif self.rect.top > height - 500 and self.waitTime < 1000:
            self.movement[1] = 0
            self.waitTime += 1

        if self.waitTime >= 150:
            self.movement[1] = 5

        if self.rect.top > height:
            self.kill()


class enemysaucer(pygame.sprite.Sprite):

    def __init__(self, x):
        pygame.sprite.Sprite.__init__(self, self.containers)
        sheet = pygame.image.load('Sprites/enemy_saucer1.png')
        self.images = []

        for i in range(0, 672, 96):
            rect = pygame.Rect((i, 0, 96, 96))
            image = pygame.Surface(rect.size)
            image = image.convert()
            colorkey = -1
            colorkey = image.get_at((10, 10))
            image.set_colorkey(colorkey, RLEACCEL)
            image.blit(sheet, (0, 0), rect)
            image = pygame.transform.scale(image, (48, 48))
            self.images.append(image)

        self.image = self.images[0]
        self.index = 0
        self.rect = self.image.get_rect()
        self.rect.center = (x, -self.rect.height)
        self.health = 10
        self.waitTime = 0
        self.fire = 1
        self.movement = [0, 0]
        self.haltpos = random.randrange(300, 510)
        self.shot = False
        self.explosion_sound = \
            pygame.mixer.Sound('Sprites/explosion.wav')
        self.explosion_sound.set_volume(0.1)

    def checkbounds(self):
        if self.rect.left < 0:
            self.rect.left = 0
            self.movement[0] = 0
            self.speed = 0
        if self.rect.right > width:
            self.rect.right = width
            self.movement[0] = 0
            self.speed = 0

    def update(self):
        self.checkbounds()
        self.autopilot()
        self.rect = self.rect.move(self.movement)

        if self.fire == 1 and self.waitTime % 10 == 1:
            self.shoot()

        if self.health <= 0:
            (x, y) = self.rect.center
            if pygame.mixer.get_init():
                self.explosion_sound.play(maxtime=1000)
            explosion(x, y,75)
            self.kill()
        self.index += 1
        self.index = self.index % 7
        self.image = self.images[self.index]
        self.image = pygame.transform.rotate(self.image, 90)
        self.images[self.index] = self.image

    def drawplayer(self):
        screen.blit(self.image, self.rect)

    def shoot(self):
        (x, y) = self.rect.center
        self.shot = enemybullet(x, y, (0, 0, 255), [0, 1], 18)

    def autopilot(self):
        if self.rect.top < height - self.haltpos:
            self.movement[1] = 3
        elif self.rect.top > height - self.haltpos and self.waitTime \
            < 1000:
            self.movement[1] = 0
            self.waitTime += 1

        if self.waitTime >= 150:
            self.movement[1] = 5

        if self.rect.top > height:
            self.kill()


class enemystation(pygame.sprite.Sprite):

    def __init__(self, x):
        pygame.sprite.Sprite.__init__(self, self.containers)
        (self.image, self.rect) = load_image('spacestation_scale.png',
                150, 150, -1)

        self.rect.center = (x, -self.rect.height)
        self.health = 60
        self.waitTime = 0
        self.fire = 1
        self.movement = [0, 0]
        self.shot = False
        self.explosion_sound = \
            pygame.mixer.Sound('Sprites/explosion.wav')
        self.explosion_sound.set_volume(0.1)
        self.rotation = 10

    def checkbounds(self):
        if self.rect.left < 0:
            self.rect.left = 0
            self.movement[0] = 0
            self.speed = 0
        if self.rect.right > width:
            self.rect.right = width
            self.movement[0] = 0
            self.speed = 0

    def update(self):
        self.checkbounds()
        self.autopilot()
        self.rect = self.rect.move(self.movement)

        if self.fire == 1 and self.waitTime % 10 == 1:
            self.shoot()

        if self.health <= 0:
            (x, y) = self.rect.center
            if pygame.mixer.get_init():
                self.explosion_sound.play(maxtime=1000)
            explosion(x, y,150)
            self.kill()

        if self.waitTime > 0:
            self.image = pygame.transform.rotate(self.image, 90)

    def drawplayer(self):
        screen.blit(self.image, self.rect)

    def shoot(self):
        (x, y) = self.rect.center
        for j in range(-12, 12):
            self.shot = enemybullet(x, y, (0, 255, 0), [j / 3.0, 1], 10)
        if self.waitTime % 2 == 1:
            enemy(random.randrange(0, 4))

        if self.waitTime % 12 == 1:
            enemysaucer(random.randrange(0, width - 50))

    def autopilot(self):
        if self.rect.top < height - 500:
            self.movement[1] = 3
        elif self.rect.top > height - 500 and self.waitTime < 1000:
            self.movement[1] = 0
            self.waitTime += 1

        if self.waitTime >= 150:
            self.movement[1] = 5

        if self.rect.top > height:
            self.kill()


class healthpack(pygame.sprite.Sprite):

    def __init__(
        self,
        x,
        y,
        health,
        ):

        pygame.sprite.Sprite.__init__(self, self.containers)
        self.health = health
        (self.image, self.rect) = load_image('healthpack.png', 40, 40,
                -1)
        self.rect.left = x
        self.rect.top = y
        self.movement = [3, 0]
        self.maxleft = self.rect.left - 20
        self.maxright = self.rect.right + 20

    def checkbounds(self):
        if self.rect.left < 0:
            self.rect.left = 0
            self.movement[0] = 0
            self.speed = 0
        if self.rect.right > width:
            self.rect.right = width
            self.movement[0] = 0
            self.speed = 0

    def update(self):
        self.checkbounds()
        self.autopilot()
        self.rect = self.rect.move(self.movement)

        if self.health <= 0 or self.rect.top > height:
            self.kill()

    def drawplayer(self):
        screen.blit(self.image, self.rect)

    def autopilot(self):
        if self.rect.right > self.maxright:
            self.movement[0] = -3
        elif self.rect.left < self.maxleft:
            self.movement[0] = 3

        self.movement[1] = 5


class bullet(pygame.sprite.Sprite):

    def __init__(
        self,
        x,
        y,
        color,
        direction=1,
        ):

        pygame.sprite.Sprite.__init__(self, self.containers)
        """self.image = pygame.Surface((2, 18), pygame.SRCALPHA, 32)

        self.image = self.image.convert_alpha()
        pygame.draw.rect(self.image, color, (0, 0, 2, 18))  # (12,225,15)
        self.rect = self.image.get_rect()
        """
        self.image,self.rect = load_image('lazer.png',5,25,-1)
        self.rect.center = (x, y - direction * 20)
        self.direction = direction

    def update(self):
        (x, y) = self.rect.center
        y -= self.direction * 20
        self.rect.center = (x, y)
        if y <= 0 or y >= height:
            self.kill()

class enemybullet(pygame.sprite.Sprite):

    def __init__(
        self,
        x,
        y,
        color,
        direction,
        speed,
        ):

        pygame.sprite.Sprite.__init__(self, self.containers)
        self.image = pygame.Surface((10, 10), pygame.SRCALPHA, 32)
        self.image = self.image.convert_alpha()
        self.col = list(color)
        for i in range(5, 0, -1):
            self.col[0] = color[0] * float(i) / 5
            self.col[1] = color[1] * float(i) / 5
            self.col[2] = color[2] * float(i) / 5
            pygame.draw.circle(self.image, tuple(self.col), (5, 5), i,
                               0)
        self.rect = self.image.get_rect()
        self.rect.center = (x, y)  # + direction[1]*20)
        self.direction = direction
        self.speed = speed

    def update(self):
        (x, y) = self.rect.center
        y += self.direction[1] * self.speed
        x += self.direction[0] * self.speed
        self.rect.center = (x, y)
        if y <= 0 or y >= height or x <= 0 or x >= width:
            self.kill()


class explosion(pygame.sprite.Sprite):

    def __init__(self, x, y,radius=-1):
        pygame.sprite.Sprite.__init__(self, self.containers)
        sheet = pygame.image.load('Sprites/enemy_explode.png')
        self.images = []
        for i in range(0, 768, 48):
            rect = pygame.Rect((i, 0, 48, 48))
            image = pygame.Surface(rect.size)
            image = image.convert()
            colorkey = -1
            colorkey = image.get_at((10, 10))
            image.set_colorkey(colorkey, RLEACCEL)

            image.blit(sheet, (0, 0), rect)
            if radius != -1:
                image = pygame.transform.scale(image,(radius,radius))
            self.images.append(image)

        self.image = self.images[0]
        self.index = 0
        self.rect = self.image.get_rect()
        self.rect.center = (x, y)

    def update(self):
        self.image = self.images[self.index]
        self.index += 1
        if self.index >= len(self.images):
            self.kill()

spo_torq = 0

gameOver = False
menuExit = False
stageStart = False
bossStage = False
gameOverScreen = False

menuselect = -1
menuhighlight = 0

wavecounter = 0
wave = 0

starfield1 = stars(1,white,50,5)
starfield2 = stars(1,(150,150,150),75,3)
starfield3 = stars(1,(75,75,75),200,1)

bullets = pygame.sprite.Group()
enemybullets = pygame.sprite.Group()
enemies = pygame.sprite.Group()
explosions = pygame.sprite.Group()
shields = pygame.sprite.Group()
drones = pygame.sprite.Group()
saucers = pygame.sprite.Group()
station = pygame.sprite.Group()
healthpacks = pygame.sprite.Group()

bullet.containers = bullets
enemybullet.containers = enemybullets
enemy.containers = enemies
explosion.containers = explosions
enemydrone.containers = drones
enemysaucer.containers = saucers
enemystation.containers = station
healthpack.containers = healthpacks

user = player()
pygame.display.set_caption('PyGalaxian')
bg_music = pygame.mixer.Sound('Sprites/bg_music.ogg')
boss_music = pygame.mixer.Sound('Sprites/boss_music.ogg')

(logoimage, logorect) = load_image('gamelogo.png', -1, -1, -1)
logorect.left = width / 2 - logorect.width / 2
logorect.top = height / 2 - logorect.height * 5 / 4

bg,bgrect = load_image('bg1.png')

while not gameOver:
    
    while not menuExit:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                menuExit = True
                gameOver = True

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_DOWN or event.key \
                    == pygame.K_UP:
                    menuhighlight += 1
                elif event.key == pygame.K_RETURN:
                    menuselect = menuhighlight % 2

        if menuselect == 0:
            stageStart = True
            menuExit = True
            bg_music.play(-1)

        elif menuselect == 1:
            menuExit = True
            gameOver = True
   
        else:
            pass

        #screen.fill(sky)
        screen.blit(bg,bgrect)
        starfield1.drawstars()
        starfield2.drawstars()
        starfield3.drawstars()
        user.drawplayer()
        screen.blit(logoimage, logorect)

        displaytext('Play', 32, width / 2 - 20, height * 3 / 4
                    - 40, white)
        displaytext('Exit', 32, width / 2 - 20, height * 3 / 4,
                    white)
        displaytext('PyGalaxian version 1.0', 12, width - 80, height - 20,
                    white)
        displaytext('Made by: Shivam Shekhar', 12, width - 80, height - 10,
                    white)

        if menuhighlight % 2 == 0:
            screen.blit(pygame.transform.scale(user.image, (25,
                        25)), [width / 2 - 100, height * 3 / 4
                        - 55, 15, 15])
        elif menuhighlight % 2 == 1:
            screen.blit(pygame.transform.scale(user.image, (25,
                        25)), [width / 2 - 100, height * 3 / 4
                        - 15, 15, 15])
        pygame.display.update()
        #clock.tick(FPS)
        line = ser.readline()

    while stageStart:
        #-----------------------------robot control--------------------------
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
        # print(spo_torq)

        #axis 3, torq sensor amplify, add z 
        j3_torq = tx - 0.735 #joystick value
        # endjdeg = enc2deg(raw_q[3] - encOffset[3],3)/180*np.pi#angle in rad
        # #fz offset:-1.5 to -3.5. 2.5 with deadzone 1.0
        # fz = fz+2.5
        # fz = (np.sign(fz) * (abs(fz) - 1.0)) if abs(fz) > 1.0 else 0
        # #small force 7
        # #print(tx," ",fz," ",np.cos(endjdeg))
        
        # j3_torq = j3_torq*np.cos(endjdeg) + fz/9.0*np.sin(endjdeg)
        # print(endjdeg)


        pred_curr[2] = torq2curr(-(np.sign(j3_torq) * (abs(j3_torq) - dead_zone[2])) if abs(j3_torq) > dead_zone[2] else 0)
        # if abs(raw_q_dot[2]) <= 10:
        #     pred_curr[2] = pred_curr[2]*(10-abs(raw_q_dot[2]))*8  + pred_curr[2]*8 #amp factor
        # else:
        pred_curr[2] = pred_curr[2]*10

        #print(pred_curr[2])
        pred_curr[2] = max(min(pred_curr[2], mt[2]), -mt[2])
        # print(j3_torq)
        # print(raw_q_dot[2])
        # print(round(pred_curr[2]))

        torque_feedforward = -400
        #torque feedforward
        if (abs(raw_q_dot[2]) <= 3 and abs(pred_curr[2]) > 0):
            pred_curr[2] = pred_curr[2] + torque_feedforward*np.sign(j3_torq)
        #*(10-abs(raw_q_dot[2]))
        #speed damper
        resistance = 0
        resistance = -raw_q_dot[2] * 0.07
        pred_curr[2] = pred_curr[2] + resistance

        #impedance k 
        k = 1.2
        cur_q_deg_j3 = enc2deg(raw_q[2] - encOffset[2], 2)
        q_diff = -90 - cur_q_deg_j3
        k_out = q_diff*k
        pred_curr[2] = pred_curr[2] + k_out
        #print(k_out)
        #print(round(pred_curr[2]))
        #print('---------------------')
        #add torque feed
        command += to_char(round(0))
        #command += to_char(round(pred_curr[2]))

        #axis 4, torque sensor amplify
        j4_torq = fy-38.6#joystick value
        pred_curr[3] = torq2curr(-(np.sign(j4_torq) * (abs(j4_torq) - dead_zone[2])) if abs(j4_torq) > dead_zone[3] else 0)
        pred_curr[3] = pred_curr[3]*-1 #amp factor
        pred_curr[3] = max(min(pred_curr[3], mt[3]), -mt[3])
        #print(j4_torq)
        #print(round(pred_curr[3]))
        command += to_char(round(pred_curr[3]))
    
        #print(command)
        ser.write(command.encode())

        j1_q = enc2deg(raw_q[0] - encOffset[0], 0)
        j4_q = enc2deg(raw_q[3] - encOffset[3], 3)

        #convert to game coord 

        #only j1: 30deg -> 120deg
        #         1200 -> 0
        angle = 0
        if j1_q < 30:
            angle = 30
        elif j1_q > 120:
            angle = 120
        else:
            angle = j1_q

        x_coord = (angle-30)*13.33 
        # user.movement[0] = x_coord
        print(x_coord)

        #--------------------------------------------------------------end robot control-------------------------------------
        #always shooting
        user.fire = 1
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                stageStart = False
                gameOver = True
            
            if event.type == pygame.KEYDOWN:
                user.trigger = 1
                if event.key == pygame.K_LEFT:
                    user.speed = -2
                elif event.key == pygame.K_RIGHT:
                    user.speed = 2
                # elif event.key == pygame.K_UP:
                #     user.fire = 1
                

            if event.type == pygame.KEYUP:
                if event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                    user.trigger = 2
                    user.speed = 0
                # if event.key == pygame.K_UP:
                #     user.fire = 0

        if wavecounter % 500 == 499 and random.randrange(0, 2) == 1 \
            and len(healthpacks) < 1:
            healthpack(random.randrange(0, width - 50), 0, 10)

        if random.randrange(0, 8) == 1 and len(enemies) < 10 \
            and (wave == 0 or wave == 5 or wave == 6 or wave == 9):
            enemy(random.randrange(0, 4))

        if random.randrange(0, 20) == 1 and len(saucers) < 3 \
            and (wave == 1 or wave == 5 or wave == 7 or wave == 8):
            enemysaucer(random.randrange(0, width - 50))

        if random.randrange(0, 30) == 21 and len(drones) < 2 \
            and (wave == 2 or wave == 4 or wave == 7 or wave == 9):
            if len(drones) > 0:
                for drone in drones:
                    if drone.rect.left < width / 2:
                        enemydrone(random.randrange(width / 2 + 60,
                                width - 60))
                    else:
                        enemydrone(random.randrange(0, width / 2
                                - 60))
            else:
                enemydrone(random.randrange(0, width - 60))

        if len(station) < 1 and (wave == 3 or wave == 10):
            enemystation(random.randrange(0, width - 60))

        if wave == 11 and len(enemies) == 0 and len(saucers) == 0 \
            and len(station) == 0 and len(drones) == 0:
            user.isautopilot = True
            bg_music.fadeout(6000)
            if user.rect.top <= -1*user.rect.height:
                wave = 12

        if wave == 12:
            bossStage = False #disable boss
            stageStart = False
            finalboss = boss()
            user.health += 80
            user.rect.left = width / 2
            user.rect.top = size[1] - 100
            user.isautopilot = False
            user.movement = [0, 0]
            boss_music.play(-1)
            #
            gameOverScreen = True
            user.won = True

        for ship in enemies:
            cpumove(ship, user)

        for enemyhit in pygame.sprite.groupcollide(enemies,
                bullets, 0, 1):
            enemyhit.health -= 1
            if enemyhit.health <= 0:
                user.kills += 1
                user.score += 1

        for dronehit in pygame.sprite.groupcollide(drones, bullets,
                0, 1):
            dronehit.health -= 1
            if dronehit.health <= 0:
                user.kills += 1
                user.score += 10

        for saucerhit in pygame.sprite.groupcollide(saucers,
                bullets, 0, 1):
            saucerhit.health -= 1
            if saucerhit.health <= 0:
                user.kills += 1
                user.score += 5

        for stationhit in pygame.sprite.groupcollide(station,
                bullets, 0, 1):
            stationhit.health -= 1
            if stationhit.health <= 0:
                user.kills += 1
                user.score += 25
                healthpack(stationhit.rect.centerx,
                            stationhit.rect.centery, 20)

        for firedbullet in pygame.sprite.spritecollide(user,
                enemybullets, 1):
            user.health -= 1

        for enemycollided in enemies:
            if pygame.sprite.collide_mask(user, enemycollided):
                user.health -= 2
                enemycollided.health -= enemycollided.health

        for dronecollided in drones:
            if pygame.sprite.collide_mask(user, dronecollided):
                user.health -= 10
                dronecollided.health -= dronecollided.health

        for saucercollided in saucers:
            if pygame.sprite.collide_mask(user, saucercollided):
                user.health -= 4
                saucercollided.health -= saucercollided.health

        for stationcollided in station:
            if pygame.sprite.collide_mask(user, stationcollided):
                user.health -= 50
                stationcollided.health -= stationcollided.health

        for health_pack in healthpacks:
            if pygame.sprite.collide_mask(user, health_pack):
                user.health += health_pack.health
                health_pack.health -= health_pack.health

        if user.health <= 0:
            gameOverScreen = True
            stageStart = False

        user.update(x_coord)
        user.checkbounds()

        #screen.fill(sky)
        screen.blit(bg,bgrect)
        starfield1.drawstars()
        starfield2.drawstars()
        starfield3.drawstars()

        if user.health > 0:
            showhealthbar(user.health, green, [100, height - 20,
                            user.health * 4, 10], 4)
        displaytext('HEALTH', 22, 50, height - 15, white)
        displaytext('Score:', 22, width - 100, 15, white)
        displaytext(str(user.score), 22, width - 35, 15, white)
        user.drawplayer()

        enemies.update()
        bullets.update()
        enemybullets.update()
        explosions.update()
        drones.update()
        saucers.update()
        station.update()
        healthpacks.update()

        bullets.draw(screen)
        enemybullets.draw(screen)
        enemies.draw(screen)
        explosions.draw(screen)
        drones.draw(screen)
        saucers.draw(screen)
        station.draw(screen)
        healthpacks.draw(screen)

        wave = storyboard(wavecounter)

        wavecounter += 1

        pygame.display.update()

        #clock.tick(FPS)
        line = ser.readline()

        moveplayer(user)

        # print (
        #     wavecounter,
        #     wave,
        #     user.kills,
        #     user.health,
        #     user.rect.left,
        #     user.movement[0],
        #     user.rect.right,
        #     )

    # while bossStage:
    #     for event in pygame.event.get():
    #         if event.type == pygame.QUIT:
    #             gameOver = True
    #             bossStage = False
    #         if event.type == pygame.KEYDOWN:
    #             user.trigger = 1
    #             if event.key == pygame.K_LEFT:
    #                 user.speed = -2
    #             elif event.key == pygame.K_RIGHT:
    #                 user.speed = 2
    #             elif event.key == pygame.K_UP:
    #                 user.fire = 1
    #             elif event.key == pygame.K_ESCAPE:
    #             	quit()


    #         if event.type == pygame.KEYUP:
    #             if event.key == pygame.K_LEFT or event.key \
    #                 == pygame.K_RIGHT:
    #                 user.trigger = 2
    #                 user.speed = 0
    #             if event.key == pygame.K_UP:
    #                 user.fire = 0

    #     bossmove(finalboss, user)

    #     for ship in enemies:
    #         cpumove(ship, user)

    #     for userbullet in bullets:
    #         if pygame.sprite.collide_mask(finalboss, userbullet):
    #             if finalboss.health > 2:
    #                 finalboss.health -= 1
    #             else:
    #                 bossStage = False
    #                 gameOverScreen = True
    #                 user.score += 200
    #                 user.won = True
    #             userbullet.kill()

    #     for enemyhit in pygame.sprite.groupcollide(enemies,
    #             bullets, 0, 1):
    #         enemyhit.health -= 1
    #         if enemyhit.health <= 0:
    #             user.kills += 1
    #             user.score += 1

    #     for dronehit in pygame.sprite.groupcollide(drones, bullets,
    #             0, 1):
    #         dronehit.health -= 1
    #         if dronehit.health <= 0:
    #             user.kills += 1
    #             user.score += 10

    #     for saucerhit in pygame.sprite.groupcollide(saucers,
    #             bullets, 0, 1):
    #         saucerhit.health -= 1
    #         if saucerhit.health <= 0:
    #             user.kills += 1
    #             user.score += 5

    #     for firedbullet in pygame.sprite.spritecollide(user,
    #             enemybullets, 1):
    #         user.health -= 1

    #     for enemycollided in enemies:
    #         if pygame.sprite.collide_mask(user, enemycollided):
    #             user.health -= 2
    #             enemycollided.health -= enemycollided.health

    #     for dronecollided in drones:
    #         if pygame.sprite.collide_mask(user, dronecollided):
    #             user.health -= 10
    #             dronecollided.health -= dronecollided.health

    #     for saucercollided in saucers:
    #         if pygame.sprite.collide_mask(user, saucercollided):
    #             user.health -= 4
    #             saucercollided.health -= saucercollided.health

    #     if user.health <= 0:
    #         gameOverScreen = True
    #         bossStage = False

    #     user.update()
    #     user.checkbounds()

    #     #screen.fill(sky)
    #     screen.blit(bg,bgrect)
    #     starfield1.drawstars()
    #     starfield2.drawstars()
    #     starfield3.drawstars()

    #     if user.health > 0:
    #         showhealthbar(user.health, green, [100, height - 20,
    #                       user.health * 4, 10], 4)
    #     displaytext('HEALTH', 22, 50, height - 15, white)

    #     if finalboss.health > 0:
    #         showhealthbar(finalboss.health, red, [100, 20,
    #                       finalboss.health * 0.8, 10], 0.8)
    #     displaytext('BOSS', 22, 50, 25, white)

    #     displaytext('Score:', 22, width - 100, 15, white)
    #     displaytext(str(user.score), 22, width - 35, 15, white)

    #     user.drawplayer()

    #     enemies.update()
    #     bullets.update()
    #     enemybullets.update()
    #     drones.update()
    #     saucers.update()
    #     explosions.update()
    #     finalboss.update()

    #     bullets.draw(screen)
    #     enemybullets.draw(screen)
    #     enemies.draw(screen)
    #     drones.draw(screen)
    #     saucers.draw(screen)
    #     explosions.draw(screen)
    #     finalboss.drawplayer()

    #     pygame.display.update()
    #     clock.tick(FPS)
    #     moveplayer(user)

    while gameOverScreen:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                gameOverScreen = False
                gameOver = True

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:
                gameOverScreen = False
                gameOver = True

        screen.fill(sky)
        starfield1.drawstars()
        starfield2.drawstars()
        starfield3.drawstars()

        if user.won == False:
            displaytext('Game Over', 26, width / 2 - 30, height
                        / 2, white)
        else:
            displaytext('Congratulations! You Won!', 26, width / 2
                        - 30, height / 2, white)

        displaytext('Your score: ', 26, width / 2 - 40, height / 2
                    + 40, white)
        displaytext(str(user.score), 26, width / 2 + 50, height / 2
                    + 43, white)
        displaytext('Press Enter to exit...', 14, width / 2 - 30,
                    height / 2 + 90, white)
        pygame.display.update()
        #clock.tick(FPS)
        line = ser.readline()

#pygame.quit()
#quit()

# time.sleep(0.1)
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
    
