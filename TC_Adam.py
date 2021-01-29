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
ser.timeout = 1          #non-block read 0.5s
ser.writeTimeout = 1     #timeout for write 0.5s
#------------------------------------------------------------------

#-----------------ModBus RTU------------------------------
RTU1 = Modbus.RTU('03', '03', '0000', '0008')
slave_3 = Modbus.gen_Slave(RTU1)
#print(slave_3.rtu)
print('Generate Slaves: succeed')
#------------------------------------------------------------------
# %%
def Adam_data_collect(port, slave, start, time_out, wait_data):
    #while True:
    try:
        port.write(bytes.fromhex(slave.rtu)) #hex to binary(byte) 
        slave.time_readings.append(time.time()-start)

        time.sleep(time_out)

        # look up the buffer for 21 bytes, which is for 8 channels data length
        if port.inWaiting() == wait_data: 
            readings = port.read(wait_data).hex()
            slave.lst_readings.append(readings)
        else: # if data is not correct, return as None
            slave.lst_readings.append(None)
            port.reset_input_buffer() 
        print('collect done')

    except Exception as e1:
        print ("communicating error " + str(e1))
    
    # if event.is_set():
    ## break


def Adam_data_analyze(slave):
    # while not ticker.wait(5):
    lst_readings = slave.lst_readings
    time_readings = slave.time_readings
    slave.lst_readings = []
    slave.time_readings = []

    arr_readings = np.array([[int(reading[i-4:i],16) for i in range(10,len(reading)-2,4)] for reading in lst_readings])
    lst_readings = list(np.sum(arr_readings, axis=0) / len(lst_readings))

    readings = []
    readings.append(time_readings[-1])
    readings.append(lst_readings)
    slave.readings.append(readings)
    print('analyze done')

# %%
try: 
    ser.open()
    ser.reset_input_buffer() #flush input buffer
    ser.reset_output_buffer() #flush output buffer
except Exception as ex:
    print ("open serial port error " + str(ex))
    exit()
start = time.time()

#while True:
for i in range(3):
    for i in range(5):
        Adam_data_collect(ser, slave_3, start, 1, 21)
    Adam_data_analyze(slave_3)

ser.close()
print(slave_3.lst_readings)
print(slave_3.time_readings)
print(slave_3.readings)
