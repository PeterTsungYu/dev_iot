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

def gen_Slave(RTU):
    slave_3 = Modbus.Slave(RTU.id, RTU.rtu)
    return slave_3
slave_3 = gen_Slave(RTU1)
#print(slave_3.rtu)
print('Generate Slaves: succeed')
#------------------------------------------------------------------
# %%
# define Threads and Events...

def buffer_check():    
    while True:
    # look up the buffer for reading
        if ser.in_waiting == (int(RTU1.data_len) * 2): # input has data with required length 
            buffer_state.set()

def ReadAdam():
    start_w = time.time()
    while True:
        while ticker.wait(WAIT_TIME_SECONDS): # execute the following while block for a countdown, and then end the loop
            ser.write(bytearray.fromhex(slave_3.rtu[0])) #hex to binary(byte) 
            slave_3.time_readings.append(time.time()-start_w)
            print(f"writing to slave_{slave_3.id}")

            buffer_state.wait() # wait for the buffer_check to set the flag 
            print(f"Reading from slave_{slave_3.id}")
            print(ser.in_waiting)
            return_data = ser.read(ser.in_waiting).hex() # return as string
            slave_3.lst_readings.append(return_data)
            #ser.reset_input_buffer() 
            buffer_state.clear()        
        
        # calc after every min, code here...
        readings = np.array([[reading[i-2:i] for i in range(4+2,len(reading)-2,2)]for reading in slave_3.lst_readings])
        slave_3.arr_readings = np.sum(readings, axis=0) / len(slave_3.lst_readings)
        slave_3.time_readings[-1]
        ## code here...

        # flush all data in attributes in slave_3 
        slave_3 = gen_Slave(RTU1)


buffer_state = threading.Event()
buffer_state.clear() # set initial event is False
buffer_gate = threading.Thread(target=buffer_check)
Adam_TC_reader = threading.Thread(target=ReadAdam)

WAIT_TIME_SECONDS = 60 # per min
ticker = threading.Event()
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

while True: # execute the following after every min countdown
    try:
        print('GUI will be here')
    except Exception as e1:
        print ("communicating error " + str(e1))

# close the port
ser.close()
print("Port closed")
print('succeed')

print(slave_3.lst_readings)
print(slave_3.time_readings)
print('end')
# %%
# verify the exported file
#np.load('./slave_1_readings.npy')