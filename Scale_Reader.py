# %%
import serial
import numpy as np
import time
import Modbus
import threading
import signal
import re

print('Import: succeed')

# %%
#-----------------Master(RPi) setting------------------------------
ser = serial.Serial()
ser.port = "/dev/ttyUSB2"

# According to Adam module spec...60 Hz, O_81
ser.baudrate = 9600
ser.bytesize = serial.EIGHTBITS # 8 bits per bytes
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
ser.parity = serial.PARITY_NONE #set parity check
#ser.timeout = 0.5          #non-block read 0.5s
#ser.writeTimeout = 0.5     #timeout for write 0.5s
#------------------------------------------------------------------
#-----------------ModBus------------------------------
slave_Scale = Modbus.Slave()
print('Generate Slaves: succeed')
#------------------------------------------------------------------
# %%
def data_collect(port, slave, start, time_out, wait_data):
    #while True:
    try:
        time.sleep(time_out) # wait for the data input to the buffer
        if port.inWaiting():
            readings = port.read(port.inWaiting()).decode('utf-8')
            #print(readings)
            slave.time_readings.append(time.time()-start)
            readings = [float(s) if s[0] != '-' else -float(s[1:]) for s in re.findall(r'[ \-][ \d]{5}\.\d', readings)]
            #print(readings)
            #print(len(readings))
            slave.lst_readings.append(readings) 
        print('collect done')
    except Exception as e1:
        print (f"{data_collect.__name__} error " + str(e1))
    finally:
        port.reset_input_buffer() # reset the buffer after each reading process


def data_analyze(slave):
    # while not ticker.wait(5):
    try:
        lst_readings = slave.lst_readings
        time_readings = slave.time_readings
        slave.lst_readings = []
        slave.time_readings = []
        #print(lst_readings)

        lst_readings = [sum(i)/len(i) for i in lst_readings] # average for 1s' data
        #print(lst_readings)
        lst_readings = round(sum(lst_readings) / len(lst_readings), 1) # average for 1min's data

        readings = []
        readings.append(time_readings[-1])
        readings.append(lst_readings)
        slave.readings.append(readings)
        print('analyze done')
    except Exception as ex:
        print (f"{data_analyze.__name__} error: " + str(ex)) 
    finally:
        slave.lst_readings = []
        slave.time_readings = []


# %%
try: 
    ser.open()
    ser.reset_input_buffer() #flush input buffer
    ser.reset_output_buffer() #flush output buffer
except Exception as ex:
    print ("open serial port error " + str(ex))
    exit()
start = time.time()
print('start')


#while True:
for i in range(2):
    for i in range(3):
        data_collect(ser, slave_Scale, start, 1, 0)
    data_analyze(slave_Scale)

ser.close()

print(slave_Scale.lst_readings)
print(slave_Scale.time_readings)
print(slave_Scale.readings)
# [[3.007049798965454, 0.0], [6.0131144523620605, 0.0]]