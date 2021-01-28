# %%
import serial
import numpy as np
import time
import Modbus
import threading

print('Import: succeed')

# %%
#-----------------Master(RPi) setting------------------------------
ser = serial.Serial()
ser.port = "/dev/ttyUSB0"

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

def gen_Slave(RTU):
    slave_3 = Modbus.Slave(RTU.id, RTU.rtu)
    return slave_3
slave_3 = gen_Slave(RTU1)
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
button_exit = threading.Event()


# Adam data collect threads
Adam_data_collect = threading.Thread(target=Modbus.Adam_data_collect, args=(button_exit, ser, slave_3, start, 1, 21))
Adam_data_analyze = threading.Thread(target=Modbus.Adam_data_analyze, args=(slave_3))

# button threads
## signal...

## Threading Class...

# %%
# main threading...
Adam_data_collect.start()
Adam_data_analyze.start()

#while True:
while True:
    try:
        print("Analysis done")
    except:
        pass
    finally:    
        ser.close()
        print(slave_3.lst_readings)
        print(slave_3.time_readings)
        print(slave_3.readings)
