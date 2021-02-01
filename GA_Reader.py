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

ser.baudrate = 9600
ser.bytesize = serial.EIGHTBITS # 8 bits per bytes
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
ser.parity = serial.PARITY_NONE #set parity check
#ser.timeout = 1          #non-block read 0.5s
#ser.writeTimeout = 1     #timeout for write 0.5s
#------------------------------------------------------------------

#-----------------ModBus RTU------------------------------
RTU_GA = '11 01 60 8E'
slave_GA = Modbus.Slave('11', RTU_GA)
#print(slave_GA.rtu)
print('Generate Slaves: succeed')
#------------------------------------------------------------------
# %%
def data_collect(port, slave, start, time_out, wait_data):
    #while True:
    try:
        port.write(bytes.fromhex(slave.rtu)) #hex to binary(byte) 
        slave.time_readings.append(time.time()-start)

        time.sleep(time_out)

        #print(port.inWaiting())
        if port.inWaiting() == wait_data: 
            readings = port.read(wait_data).hex()
            #print(len(readings))
            slave.lst_readings.append(readings)
        else: # if data is not correct, return as None
            slave.lst_readings.append(None)
            port.reset_input_buffer() 
        print('collect done')
    except Exception as e1:
        print ("communicating error " + str(e1))


def data_analyze(slave):
    # while not ticker.wait(5):
    lst_readings = slave.lst_readings
    time_readings = slave.time_readings
    slave.lst_readings = []
    slave.time_readings = []

    arr_readings = np.array(
        [[int(readings[i:i+4],16)/100 for i in range(8,20,4)] 
        + [int(readings[24:28],16)/100] 
        + [int(readings[-12:-8],16)/100] 
        + [(lambda i: ((i[0]*256+i[1]+i[2])*256+i[3])/100)([int(readings[i:i+2],16) for i in range(-20,-12,2)])] 
        for readings in lst_readings]
        )
    #print(arr_readings)
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
        data_collect(ser, slave_GA, start, 1, 31)
    data_analyze(slave_GA)

ser.close()

print(slave_GA.lst_readings)
print(slave_GA.time_readings)
print(slave_GA.readings)
