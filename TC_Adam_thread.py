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
ser.port = "/dev/ttyUSB1"

# According to Adam module spec...60 Hz, O_81
ser.baudrate = 19200
ser.bytesize = serial.EIGHTBITS # 8 bits per bytes
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
ser.parity = serial.PARITY_NONE #set parity check
#ser.timeout = 0.5          #non-block read 0.5s
#ser.writeTimeout = 0.5     #timeout for write 0.5s
#------------------------------------------------------------------

#-----------------ModBus RTU------------------------------
RTU1 = Modbus.RTU('03', '03', '0000', '0008')
slave_3 = Modbus.Slave('03', RTU1.rtu)
#print(slave_3.rtu)
print('Generate Slaves: succeed')
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

# [4.12, [0.0, 0.0, 0.0, 0.0, 0.0, 19.91, 1370.0, 1370.0]]
# [4.13, [22.53, 22.59, 22.64, 22.73, 22.78, 1370.0, 1370.0, 1370.0]