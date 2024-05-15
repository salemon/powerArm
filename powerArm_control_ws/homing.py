import serial
import time
STOP = '0'
START = '2'
HOME = '1'

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


#clear junk message stored in the UART3 send buffer
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
command = '#' + HOME + VEL + to_char(0) + to_char(0) + to_char(0) + to_char(124) + "\r"
print(command.encode())
ser.write(command.encode())
ser.close()
