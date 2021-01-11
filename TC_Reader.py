# %%
import serial
import numpy as np
import time
import subprocess

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
class Slave:     
    def __init__(self, idno, rtu_r):
        self.id = idno # id number of slave
        self.rtu = rtu_r # rtu sent by master
        self.lst_readings = [] # record readings
        self.time_readings = [] # record time
        self.arr_readings = np.array([]) # for all data 

slave_1 = Slave(1, '01 03 00 8A 00 01 A5 E0') #Body Temp
slave_2 = Slave(2, '02 03 00 8A 00 01 A5 D3') #Output Temp

slaves = [slave_1, slave_2]
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
start_r = 0  
for i in range(5):
    try:
        for slave in slaves:

            if start_w == 0:
                start_w = time.time()

            #write 8 byte data
            print(f"write to slave {slave.id}")
            ser.write(bytes.fromhex(slave.rtu)) #hex to binary(byte) 
            
            #wait 0.5s
            time.sleep(0.5)
            # record the time of reading from slaves
            slave.time_readings.append(time.time()-start_w)

            if start_r == 0:
                start_r = time.time()

            #read 8 byte data
            if ser.inWaiting(): # look up the buffer for reading
                # read the exact data size in the buffer
                # convert the byte-formed return to hex 
                return_data = ser.read(ser.inWaiting()).hex()
                #print(return_data)
                #print(type(return_data))

                # extract the reading value from the str and convert from hex to decimal(int)
                reading_value = int(return_data[-8:-4], 16)
                print(f"reading_value from slave {slave.id}: {reading_value}")
                slave.lst_readings.append(reading_value)

            #wait 0.5s
            time.sleep(0.5)
            print(time.time() - start_r)
        
    except Exception as e1:
        print ("communicating error " + str(e1))

# close the port
ser.close()
print("Port closed")

# %%
# export to files
for slave in slaves:
    slave.arr_readings = np.concatenate(
        (np.array(slave.time_readings).reshape(-1, 1), 
        np.array(slave.lst_readings).reshape(-1, 1)),
        axis=1
        )
    np.save(f'slave_{slave.id}_readings.npy', slave.arr_readings)

print("Write to *.npy")