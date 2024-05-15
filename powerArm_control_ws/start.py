import serial
import time
STOP = '0'
START = '2'
HOME = '1'
HALT = '3'

TORQ = '0'
VEL = '1'
POS = '2'

def to_char(value):
    chr_val = str(abs(value))
    num_zero = 4-len(chr_val)
    zero_ph = ""
    for i in range(num_zero):
        zero_ph = zero_ph+"0"

    if value < 0:
        return "-" + zero_ph + chr_val
    else:
        return "+" + zero_ph + chr_val

def RPM_to_RADS(rpm):
    return rpm*0.104719755/42.6
def RADS_to_RPM(rads):
    return (rads*42.6)/0.104719755

def TDeg_to_Rad(deg):
    return (deg/10.0)*0.0175


#clear junk message stored in the UART3 send buffer
ser = serial.Serial('/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0668FF535156827867105244-if01', baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS, timeout=0.5)
print("connected to: " + ser.portstr)
junk = ser.readall()
print(junk)
ser.close()
print('junkabv')

#send commands
ser = serial.Serial('/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0668FF535156827867105244-if01', baudrate=115200, stopbits=1, parity=serial.PARITY_NONE,  bytesize=serial.EIGHTBITS)
print("connected to: " + ser.portstr)
command = '#' + START + VEL + to_char(0) + to_char(0) + to_char(0) + to_char(0) + "\r"
print(command)
ser.write(command.encode())
try:
    while True:
        #----------get data-------------
        line = ser.readline()
        data = line.decode("utf-8") # in sequence pos, vel, torque
        print(line)
        command = '#' + START + VEL + to_char(0) + to_char(-5) + to_char(0) + to_char(0) + "\r"
        ser.write(command.encode())


# position = 0
# try:
#     while True:
#         #start = time.perf_counter()
#         line = ser.readline()
#         data = line.decode("utf-8")
#         # data = data[:len(data) - 1]
#         print(data)
#         #print(time.perf_counter() - start)
#         position = position + 50
#         command = START + POS + to_char(position) + "\r"
#         print(command)
#         ser.write(command.encode())

# speed = 0
# try:
#     while True:
#         #start = time.perf_counter()
#         #----------get data-------------
#         line = ser.readline()
#         data = line.decode("utf-8")
#         data = data[:len(data) - 1]
#         result = data.split()
#         position = int(result[0])
#         velocity = int(result[1])

#         #-----------convert-------    
#         position = TDeg_to_Rad(position)
    
#         velocity = RPM_to_RADS(velocity)
#         print(f"{position:.4f}", end=", ")
#         print(f"{velocity:.4f}")
#         #print(time.perf_counter() - start)
#         speed = speed + 1

#         command = START + VEL + to_char(speed) + "\r"
#         #print(command)
#         ser.write(command.encode())

except KeyboardInterrupt:
    time.sleep(0.1)
    command = '#' + HALT + VEL + to_char(0) + to_char(0) + to_char(0) + to_char(0) + "\r"
    print(command)
    ser.write(command.encode())
    ser.close()
    print("Serial Port Closed")
    raise
