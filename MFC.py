# %%
import serial
import numpy as np
import time
import Modbus
import threading
import re
import sqlite3
conn = sqlite3.connect('./SQlite/test_DB.db')

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

#-----------------ModBus------------------------------
slave_MFC = Modbus.Slave()
print('Generate Slaves: succeed')
#------------------------------------------------------------------
# %%
def data_collect(port, slave, start, time_out, wait_data):
    #while True:
    try:
        port.write(b'a\r') #string to binary(byte) 
        slave.time_readings.append(time.time()-start)

        time.sleep(time_out)
        
        print(port.inWaiting())
        if port.inWaiting() == wait_data:
            readings = port.read(port.inWaiting()).decode('utf-8')
            #print(readings)
            slave.lst_readings.append(readings)
            print('collect done')
    except Exception as e1:
        print (f"{data_collect.__name__} error " + str(e1))
    finally:
        port.reset_input_buffer() # reset the buffer after each reading process


def data_analyze(slave):
    try:
        # while not ticker.wait(5):
        lst_readings = slave.lst_readings
        time_readings = slave.time_readings
        slave.lst_readings = []
        slave.time_readings = []

        arr_readings = np.array(
            [
                (lambda i: [float(s) if s[0] != '-' else -float(s[1:]) for s in re.findall(r'[ +\-][\d.]{6}', i)])(readings) 
                for readings in lst_readings
            ]
            )
        #print(arr_readings)
        lst_readings = tuple(np.round(np.sum(arr_readings, axis=0) / len(lst_readings), 2))
        readings = tuple(time_readings[-1:]) + lst_readings
        print(readings)
        conn.execute(
            "INSERT INTO MFC(Time, Pressure, Temper, VolFlow, MassFlow, Setpoint) VALUES (?,?,?,?,?,?);", 
            readings
            )
        slave.readings.append(readings)
        print('analyze done')
    except Exception as ex:
        print (f"{data_analyze.__name__} error: " + str(ex)) 
    finally:
        slave.lst_readings = []
        slave.time_readings = []

# %%
try: 
    ser.open()
    ser.reset_input_buffer() #flush input buffer
    ser.reset_output_buffer() #flush output buffer
except Exception as ex:
    print ("open serial port error " + str(ex))
    exit()
start = time.time()
print('start')


#while True:
for i in range(5):
    for i in range(3):
        data_collect(ser, slave_MFC, start, 1, 49)
    data_analyze(slave_MFC)

ser.close()
conn.commit()
conn.close()


print(slave_MFC.lst_readings)
print(slave_MFC.time_readings)
print(slave_MFC.readings)
# [(2.004014492034912, 1.0, 25.11, 0.0, 0.0, 0.0), (5.0139899253845215, 1.0, 25.12, 0.0, 0.0, 0.0), (8.021106719970703, 1.0, 25.12, 0.0, 0.0, 0.0), (11.028206825256348, 1.0, 25.12, 0.0, 0.0, 0.0), (14.03491497039795, 1.0, 25.12, 0.0, 0.0, 0.0)]