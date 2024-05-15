import serial
import time

TOP = '0'
START = '2'
HOME = '1'
HALT = '3'

#sport is a variable passed by GUI by exec
ser = serial.Serial(sport, baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS,)
print("connected to: " + ser.portstr)

command = "#" + HOME

ser.write(command.encode())
ser.readline()
ser.close()