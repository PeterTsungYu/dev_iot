# %%
import serial
import numpy as np
import time
import Modbus
import threading
import sys

print('Import: succeed')

# %%
#-----------------Master(RPi) setting------------------------------
ser = serial.Serial()
ser.port = "/dev/ttyUSB0"

# 19200 bps, 60 Hz, O_81
ser.baudrate = 19200
ser.bytesize = serial.EIGHTBITS # 8 bits per bytes
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
ser.parity = serial.PARITY_NONE #set parity check
ser.timeout = 0.5          #non-block read 0.5s
ser.writeTimeout = 0.5     #timeout for write 0.5s
#------------------------------------------------------------------

#-----------------ModBus RTU------------------------------
RTU1 = Modbus.RTU('03', '03', '0000', '0008')
slave_3 = Modbus.gen_Slave(RTU1)
#print(slave_3.rtu)
print('Generate Slaves: succeed')
#------------------------------------------------------------------
# %%
# define Threads and Events...

def buffer_check():   
    while True:
    # look up the buffer for reading
        if not ticker.wait(1): # wait for 1s
            buffer_read.set()

def ReadAdam():
    start_w = time.time()
    while True:
        buffer_write.wait()
        ser.write(bytearray.fromhex(slave_3.rtu[0])) #hex to binary(byte) 
        slave_3.time_readings.append(time.time()-start_w)
        print(f"writing to slave_{slave_3.id}")
        buffer_write.clear()

        buffer_read.wait() # wait for the buffer_check to set the flag 
        print(f"Reading from slave_{slave_3.id}")
        return_data = ser.read(ser.in_waiting).hex() # return as string
        slave_3.lst_readings.append(return_data)
        #ser.reset_input_buffer() 
        buffer_read.clear()
        buffer_write.set()
                    
def data_analyze():
    lst_readings = []
    time_readings = []
    while not ticker.wait(WAIT_TIME_SECONDS):
        lst_readings = slave_3.lst_readings
        time_readings = slave_3.time_readings
        slave_3 = gen_Slave(RTU1) # flush the slave attributes
        # calc after every min, code here...
        '''
        readings = np.array([[reading[i-2:i] for i in range(4+2,len(reading)-2,2)]for reading in slave_3.lst_readings])
        slave_3.arr_readings = np.sum(readings, axis=0) / len(slave_3.lst_readings)
        slave_3.time_readings[-1]
        ## code here...

        # flush all data in attributes in slave_3 
        slave_3 = gen_Slave(RTU1)
        '''

# RPi buffer events
buffer_read = threading.Event()
buffer_read.clear() # set initial event is False
buffer_write = threading.Event()
buffer_write.set() # set initial event is True
buffer_gate = threading.Thread(target=buffer_check)
# count down events
ticker = threading.Event()

# Adam threads
Adam_TC_reader = threading.Thread(target=ReadAdam)
Adam_TC_reader.daemon = True

# %%
# open ports on RPi
try: 
    # open the port
    ser.open()
    ser.reset_input_buffer() #flush input buffer
    ser.reset_output_buffer() #flush output buffer
except Exception as ex:
    print ("open serial port error " + str(ex))
    exit()

# %%
# Main thread...

# flow_rate = 18 # [g/min]
# steady-state recording 
buffer_gate.start()
Adam_TC_reader.start()

if not ticker.wait(5):
    pass
#while True: # execute the following after every min countdown
    '''
    try:
        pass
        #print('GUI will be here')
    except Exception as e1:
        print ("communicating error " + str(e1))
    '''

# close the port
ser.close()
sys.exit()
print("Port closed")
print('succeed')

print(slave_3.lst_readings)
print(slave_3.time_readings)
print('end')

# %%
# verify the exported file
#np.load('./slave_1_readings.npy')