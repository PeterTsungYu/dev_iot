# %%
import serial
import numpy as np
import time
import subprocess

display('succeed')

# %%
#-----------------Master(RPi) setting------------------------------
# for setting up the controller head physically, the baudrate, bytesize, stopbits, parity, Id number need to be the same setting as in here

# Initiate an serial instance by call the class: Serial() 
## port of the master (RPi)
ser = serial.Serial()

# set the USB port name (acquired by $ dmesg | grep tty)
ser.port = "/dev/ttyUSB0"

# 9600 bps
ser.baudrate = 19200
# O_81
ser.bytesize = serial.EIGHTBITS # 8 bits per bytes
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
ser.parity = serial.PARITY_EVEN #set parity check
ser.timeout = 0.5          #non-block read 0.5s
ser.writeTimeout = 0.5     #timeout for write 0.5s
#ser.xonxoff = False    #disable software flow control. False as default
#ser.rtscts = False     #disable hardware (RTS/CTS) flow control. False as default
#ser.dsrdtr = False     #disable hardware (DSR/DTR) flow control. False as default
#------------------------------------------------------------------

#-----------------ModBus RTU------------------------------
# master reads from slaves via RTU protocol
## RTU is in Hex  
## IDno(1byte/2hex) + func_code(1byte/2hex) + data_site(2byte/4hex) + data_entry(2byte/4hex) + CRC(2byte/4hex)
## func_code = 03, read data
## data_site = 008A (depend on the manual, specifically for FY controller head)
## CRC is caculated from the string ahead. After calculation, it needs to be swapped.         
    ## i.e. for slave_1, '01 03 00 8A 00 01' for the input to calculate the CRC, which the result is E0A5. Swap it for A5E0.
    ## https://crccalc.com/
class Slave:     
    def __init__(self, idno, rtu):
        self.id = idno # id number of slave
        self.rtu = rtu # rtu sent by master
        self.lst_readings = {'r_PV':[], 'r_SV':[]} # record readings
        self.time_readings = {'r_PV':[], 'r_SV':[]} # record time
        self.arr_readings = np.array([]) # for all data 

slave_1 = Slave(1, {
    'r_PV':'01 03 00 8A 00 01 A5 E0', 
    'r_SV':'01 03 00 00 00 01 84 0A'
    }) #Body Temp with PID controller and SSD relay
slave_2 = Slave(2, {
    'r_PV':'02 03 00 8A 00 01 A5 D3',
    'r_SV':'02 03 00 00 00 01 84 39'
    }) #Output Temp 

slaves = [slave_1, slave_2]

display('succeed')
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
# T_set = input(), from user input
flow_rate = 18 # [g/min]
# write to slave 
# steady-state recording
# user input to terminate the program 
for i in range(3):
    try:
        for slave in slaves:
            for value in ['r_PV', 'r_SV']:
                if start_w == 0:
                    start_w = time.time()

                #write 8 byte data
                ser.write(bytes.fromhex(slave.rtu[value])) #hex to binary(byte) 
                
                # record the time of reading from slaves
                slave.time_readings[value].append(time.time()-start_w)
                print(f"writing {value} to slave_{slave.id}")

                #wait[s] for reading from slave
                time.sleep(0.1)

                #read 8 byte data
                if ser.inWaiting(): # look up the buffer for reading
                    # read the exact data size in the buffer
                    # convert the byte-formed return to hex 
                    return_data = ser.read(ser.inWaiting()).hex()
                    #print(return_data)
                    #print(type(return_data))

                    # extract the reading value from the str and convert from hex to decimal(int)
                    reading_value = int(return_data[-8:-4], 16)
                    print(f"reading {value} from slave_{slave.id}: {reading_value}")
                    slave.lst_readings[value].append(reading_value)
        
    except Exception as e1:
        print ("communicating error " + str(e1))

# close the port
ser.close()
print("Port closed")

display('succeed')
# %%
# export to files
for slave in slaves:
    slave.arr_readings = np.concatenate(
        (np.array(slave.time_readings).reshape(-1, 1), 
        np.array(slave.lst_readings).reshape(-1, 1)),
        axis=1
        ).squeeze()
    np.save(f'slave_{slave.id}_readings.npy', slave.arr_readings)

print("Write to *.npy")

display('succeed')
# %%
# verify the exported file
np.load('./slave_1_readings.npy')
# %%
