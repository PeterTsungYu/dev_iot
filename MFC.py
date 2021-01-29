# %%
import serial
import numpy as np
import time
import Modbus
import threading
import signal

print('Import: succeed')

# %%
#-----------------Master(RPi) setting------------------------------
ser = serial.Serial()
ser.port = ""

# According to Adam module spec...60 Hz, O_81
ser.baudrate = 19200
ser.bytesize = serial.EIGHTBITS # 8 bits per bytes
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
ser.parity = serial.PARITY_NONE #set parity check
#ser.timeout = 0.5          #non-block read 0.5s
#ser.writeTimeout = 0.5     #timeout for write 0.5s
#------------------------------------------------------------------

# %%
try: 
    ser.open()
    ser.reset_input_buffer() #flush input buffer
    ser.reset_output_buffer() #flush output buffer
except Exception as ex:
    print ("open serial port error " + str(ex))
    exit()
start = time.time()

try:
    print(ser.inWaiting())
    if ser.inWaiting(): 
        readings = ser.read(ser.inWaiting())
    print(readings)
except Exception as e1:
    print ("Error " + str(e1))

ser.close()