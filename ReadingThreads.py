# %%
import serial
import RPi.GPIO as GPIO
import numpy as np
import time
import Modbus
import threading
import signal
import re
import sqlite3
conn = sqlite3.connect('./SQlite/test_DB.db')


print('Import: succeed')

# %%
#-----------------Serial port setting------------------------------
## device ID
ADAM_TC_id = '03'
GA_id = '11'
MFC_id = 'a'

## RS485
ADAM_TC_port = serial.Serial(port=input('ADAM_TC port: '), baudrate=19200, bytesize=EIGHTBITS, stopbits=STOPBITS_ONE, parity=PARITY_NONE)
ADAM_TC_RTU = Modbus.RTU(ADAM_TC_id, '03', '0000', '0008')
ADAM_TC_slave = Modbus.Slave(ADAM_TC_id, ADAM_TC_RTU.rtu)

## RS232
GA_port = serial.Serial(port=input('GA port: '), baudrate=9600, bytesize=EIGHTBITS, stopbits=STOPBITS_ONE, parity=PARITY_NONE)
GA_RTU = '11 01 60 8E'
GA_slave = Modbus.Slave(GA_id, GA_RTU)

MFC_port = serial.Serial(port=input('MFC port: '), baudrate=19200, bytesize=EIGHTBITS, stopbits=STOPBITS_ONE, parity=PARITY_NONE)
GA_RTU = MFC_id + '\r'
GA_slave = Modbus.Slave(GA_id, GA_RTU)

# Scale USB
Scale_port = serial.Serial(port=input('Scale port: '), baudrate=9600, bytesize=EIGHTBITS, stopbits=STOPBITS_ONE, parity=PARITY_NONE)
Scale_slave = Modbus.Slave()

#-----GPIO port setting------------------------------
## DFM

print('Serial port setting: succeed')
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

# %%
# define Threads and Events...
# count down events
ticker = threading.Event()

# Keyboard interrupt to kill all the threads
kbinterrupt_event = threading.Event()
def signal_handler(signum, frame):
    kbinterrupt_event.set()
signal.signal(signal.SIGINT, signal_handler) # Keyboard interrupt to stop the program

# Adam data collect threads
Adam_data_collect = threading.Thread(
    target=Modbus.Adam_data_collect, 
    args=(kbinterrupt_event, ser, slave_3, start, 1, 21,),
    )
Adam_data_analyze = threading.Thread(
    target=Modbus.Adam_data_analyze, 
    args=(kbinterrupt_event, ticker, slave_3,),
    )

# %%
# main threading...
Adam_data_collect.start()
Adam_data_analyze.start()

#while True:
try:
    while not kbinterrupt_event.isSet():
        pass
except:
    pass
finally:    
    print('kill main thread')