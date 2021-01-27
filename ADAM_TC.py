# %%
import serial
import numpy as np
import time
from crccheck.crc import Crc16Modbus

print('succeed')

# %%
#-----------------Master(RPi) setting------------------------------
# for setting up the controller head physically, the baudrate, bytesize, stopbits, parity, Id number need to be the same setting as in here

# Initiate an serial instance by call the class: Serial() 
## port of the master (RPi)
ser = serial.Serial()
# set the USB port name (acquired by $ dmesg | grep tty)
ser.port = "/dev/ttyUSB0"

# According to Adam module spec...
# 19200 bps
# 60 Hz
ser.baudrate = 19200
# O_81
ser.bytesize = serial.EIGHTBITS # 8 bits per bytes
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
ser.parity = serial.PARITY_NONE #set parity check
#ser.timeout = 0.5          #non-block read 0.5s
#ser.writeTimeout = 0.5     #timeout for write 0.5s

#ser.xonxoff = False    #disable software flow control. False as default
#ser.rtscts = False     #disable hardware (RTS/CTS) flow control. False as default
#ser.dsrdtr = False     #disable hardware (DSR/DTR) flow control. False as default
#------------------------------------------------------------------

#-----------------ModBus RTU------------------------------
# master reads from slaves via RTU protocol
## RTU is in Hex  
## IDno(1byte/2hex) + func_code(1byte/2hex) + data_site(2byte/4hex) + data_entry(2byte/4hex) + CRC(2byte/4hex)
## func_code = 03, to read data
## data_site = 008A (depend on the manual, specifically for FY controller head)
## CRC is caculated from the string ahead. After calculation, it needs to be swapped.         
    ## i.e. for slave_1, '01 03 00 8A 00 01' for the input to calculate the CRC, which the result is E0A5. Swap it for A5E0.
    ## https://crccalc.com/
class Slave:     
    def __init__(self, idno, rtu):
        self.id = idno # id number of slave
        self.crc = Crc16Modbus.calchex(bytearray.fromhex(rtu))
        self.rtu = rtu + self.crc[-2:] + self.crc[:2]  # rtu sent by master
        self.lst_readings = {'temp_0':[]} # record readings
        self.time_readings = {'temp_0':[]} # record time
        self.arr_readings = np.array([]) # for all data 


def gen_Slave():
    slave_3 = Slave(3, {
        'temp_0':'030300000001 85e8'
        })
    return slave_3

slave_3 = gen_Slave()
print('succeed')
#------------------------------------------------------------------

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
#flow_rate = 18 # [g/min]
# write to slave_3 
# steady-state recording
# user input to terminate the program 

for i in range(5):
    try:
        if start_w == 0:
            start_w = time.time()
        
        #write 8 byte data
        ser.write(bytes.fromhex(slave_3.rtu['temp_0'])) #hex to binary(byte) 
        
        # record the time of reading from slaves
        slave_3.time_readings['temp_0'].append(time.time()-start_w)
        print(f"writing temp_0 to slave_{slave_3.id}")

        #wait[s] for reading from slave_3
        time.sleep(nap)

        #read 8 byte data
        if ser.inWaiting(): # look up the buffer for reading
            # read the exact data size in the buffer
            # convert the byte-formed return to hex 
            return_data = ser.read(ser.inWaiting()).hex()
            print(return_data)
            #print(type(return_data))

            # extract the reading temp_0 from the str and convert from hex to decimal(int)
            reading_value = int(return_data[-8:-4], 16)
            print(f"reading temp_0 from slave_{slave_3.id}: {reading_value}")
            slave_3.lst_readings['temp_0'].append(reading_value)
        
        #! end condition
        # code here, modify to be a variable in the for loop
        # Otherwise, when the machine shutdown, SV = none. Then infinite loop.
    
        count += 1
        if count % chunk == 0:
            # export to files
            slave_3.arr_readings = np.concatenate(
                (np.array(slave_3.time_readings).reshape(-1, 1), 
                np.array(slave_3.lst_readings).reshape(-1, 1)),
                axis=1).squeeze()
            np.save(f'slave_{slave_3.id}_readings_{count}.npy', slave_3.arr_readings)
        
        # flush all data in attributes in slave_3 
        slaves = gen_Slave()
    #! add line to make sure you get the last small chunk
    # code here
    #  
    except Exception as e1:
        print ("communicating error " + str(e1))

# close the port
ser.close()
print("Port closed")
print('succeed')

# %%
# verify the exported file
#np.load('./slave_1_readings.npy')
# %%
