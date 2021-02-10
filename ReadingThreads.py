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

print('Import: succeed')

#-----------------Serial port setting------------------------------
lst_port = []
## device ID
ADAM_TC_id = '03'
GA_id = '11'
MFC_id = 'a'

## RS485
### set the baudrate of ADAM to 19200
RS485_port = serial.Serial(
    port='/dev/ttyUSB3', # input('RS485 port: ') 
    baudrate=19200, 
    bytesize=8, 
    stopbits=1, 
    parity='N'
    )
lst_port.append(RS485_port)
ADAM_TC_RTU = Modbus.RTU(ADAM_TC_id, '03', '0000', '0008')
ADAM_TC_slave = Modbus.Slave(ADAM_TC_id, ADAM_TC_RTU.rtu)

## RS232
### set the baudrate of GA & MFC to 9600
RS232_port = serial.Serial(
    port='/dev/ttyUSB4', # input('RS232 port: ') 
    baudrate=9600, 
    bytesize=8, 
    stopbits=1, 
    parity='N'
    )
lst_port.append(RS232_port)
GA_RTU = '11 01 60 8E'
GA_slave = Modbus.Slave(GA_id, GA_RTU)

MFC_RTU = MFC_id + '\r'
MFC_slave = Modbus.Slave(MFC_id, MFC_RTU)

# Scale USB
Scale_port = serial.Serial(
    port='/dev/ttyUSB5', # input('Scale port: ') 
    baudrate=9600, 
    bytesize=8, 
    stopbits=1, 
    parity='N'
    )
lst_port.append(Scale_port)
Scale_slave = Modbus.Slave()

#-----GPIO port setting----------------------------------------------------------------
## DFM
# read High as 3.3V
channel_DFM = 18
GPIO.setmode(GPIO.BCM)
DFM_slave = Modbus.Slave()

print('Port setting: succeed')

#-------------------------define Threads and Events-----------------------------------------
start = time.time()
sample_time = 30
lst_thread = []

## count down events
ticker = threading.Event()

## Keyboard interrupt event to kill all the threads
kbinterrupt_event = threading.Event()
def signal_handler(signum, frame):
    kbinterrupt_event.set()
signal.signal(signal.SIGINT, signal_handler) # Keyboard interrupt to stop the program

## RS485
Adam_data_collect = threading.Thread(
    target=Modbus.Adam_data_collect, 
    args=(kbinterrupt_event, RS485_port, ADAM_TC_slave, start, 1, 21,),
    )
Adam_data_analyze = threading.Thread(
    target=Modbus.Adam_data_analyze, 
    args=(kbinterrupt_event, ticker, sample_time, ADAM_TC_slave, './SQlite/test_DB.db',),
    )
lst_thread.append(Adam_data_collect)
lst_thread.append(Adam_data_analyze)

## RS232
def RS232_data_collect(kb_event, port):
    while not kb_event.isSet():
        Modbus.GA_data_collect(port, GA_slave, start, 1, 31)
        #Modbus.MFC_data_collect(port, MFC_slave, start, 1, 49)
    port.close()
    print('kill GA_data_collect')
    print('kill MFC_data_collect')
RS232_data_collect = threading.Thread(
    target=RS232_data_collect, 
    args=(kbinterrupt_event, RS232_port,)
    )
GA_data_analyze = threading.Thread(
    target=Modbus.GA_data_analyze, 
    args=(kbinterrupt_event, ticker, sample_time, GA_slave, './SQlite/test_DB.db',),
    )
MFC_data_analyze = threading.Thread(
    target=Modbus.MFC_data_analyze, 
    args=(kbinterrupt_event, ticker, sample_time, MFC_slave, './SQlite/test_DB.db',),
    )
lst_thread.append(RS232_data_collect)
lst_thread.append(GA_data_analyze)
lst_thread.append(MFC_data_analyze)


## Scale USB
Scale_data_collect = threading.Thread(
    target=Modbus.Scale_data_collect, 
    args=(kbinterrupt_event, Scale_port, Scale_slave, start, 1,),
    )
Scale_data_analyze = threading.Thread(
    target=Modbus.Scale_data_analyze, 
    args=(kbinterrupt_event, ticker, sample_time, Scale_slave, './SQlite/test_DB.db',),
    )
lst_thread.append(Scale_data_collect)
lst_thread.append(Scale_data_analyze)

## GPIO
# Edge detection
def DFM_data_collect(channel_DFM):
    DFM_slave.time_readings.append(time.time())
DFM_data_analyze = threading.Thread(
    target=Modbus.DFM_data_analyze, 
    args=(kbinterrupt_event, ticker, start, 60, DFM_slave, './SQlite/test_DB.db',),
    )
lst_thread.append(DFM_data_analyze)

#-------------------------Open ports--------------------------------------
try:
    for port in lst_port: 
        if not port.is_open:
            port.open() # code here........
        port.reset_input_buffer() #flush input buffer
        port.reset_output_buffer() #flush output buffer
    print('serial ports open')
    
    GPIO.setup(channel_DFM, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    print('GPIO ports open')
    
except Exception as ex:
    print ("open serial port error: " + str(ex))
    for port in lst_port: 
        port.close()
    exit() 

#-------------------------Sub Threadingggg-----------------------------------------
for subthread in lst_thread:
    subthread.start()

GPIO.add_event_detect(channel_DFM, GPIO.RISING, callback=DFM_data_collect)

#-------------------------Main Threadingggg-----------------------------------------
try:
    while not kbinterrupt_event.isSet():
        if not ticker.wait(sample_time):
            print(f'Sampling at {time.time()-start}')
            print("=="*30)
        pass
except KeyboardInterrupt: 
    print(f"Keyboard Interrupt in main thread!")
    print("=="*30)
except Exception as ex:
    print ("Main threading error: " + str(ex))    
    print("=="*30)
finally:
    GPIO.cleanup()
    print(f"Program duration: {time.time() - start}")
    print('kill main thread')

'''
conn = sqlite3.connect('./SQlite/test_DB.db')
print(MFC_slave.readings)
conn.executemany(
    "INSERT INTO MFC(Time, Pressure, Temper, VolFlow, MassFlow, Setpoint) VALUES (?,?,?,?,?,?);", 
    MFC_slave.readings
    )
conn.commit()
conn.close()
'''