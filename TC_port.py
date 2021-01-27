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

def gen_Slave():
    slave_3 = Modbus.Slave('03', RTU1.rtu)
    return slave_3

slave_3 = gen_Slave()
print(slave_3.rtu)
print('Generate Slaves: succeed')
#------------------------------------------------------------------
# %%
def buffer_check():
    # set initial event is False
    if read_state.isSet(): 
        read_state.clear() # read, wait
    if write_state.isSet(): 
        write_state.set() # write, first       
    while True:
    # look up the buffer for reading
        if ser.in_waiting == (int(RTU1.data_len) * 2): # input has data with required length 
            read_state.set()
            write_state.set()

def ReadSlave():
    while True:
        read_state.wait()
        print('read')
        print(ser.in_waiting)
        return_data = ser.read(ser.in_waiting).hex() # return as string
        slave_3.lst_readings.append(return_data)
        #ser.reset_input_buffer() 
        read_state.clear()

def WriteSlave():
    while True:
        if start_w == 0:
            start_w = time.time()
        
        write_state.wait()
        ser.write(bytearray.fromhex(slave_3.rtu[0])) #hex to binary(byte) 
        slave_3.time_readings.append(time.time()-start_w)
        print(f"writing to slave_{slave_3.id}")
        write_state.clear()

read_state = threading.Event()
write_state = threading.Event()
reader = threading.Thread(target=ReadSlave)
writer = threading.Thread(target=WriteSlave)
valve = threading.Thread(target=buffer_check)
# %%
try: 
    # open the port
    ser.open()
    ser.reset_input_buffer() #flush input buffer
    ser.reset_output_buffer() #flush output buffer

except Exception as ex:
    print ("open serial port error " + str(ex))
    exit()

start_w = 0
count = 0
chunk = 10
nap = 1
# flow_rate = 18 # [g/min]
# steady-state recording 

# %%
valve.start()
reader.start()
writer.start()

for i in range(5):
    try:
        print('round complete')

        # flush all data in attributes in slave_3 
        #slaves = gen_Slave()

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
# %%
