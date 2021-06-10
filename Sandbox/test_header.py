# %%
import serial
import RPi.GPIO as GPIO
import numpy as np
import time
from datetime import datetime
import Modbus
import threading
import re

print('Import: succeed')

#-----------------Serial port setting------------------------------
# (Optional) Set the USB devices to have a default name
RS232_Header_Vap_port_path = '/dev/ttyUSB0'
RS232_pump_port_path = '/dev/ttyUSB'
lst_port = []

## device ID
Header_Vap_id = '01'
Pump_id = ''

#-----------------Serial port instances------------------------------
## RS232
### set the baudrate of Header_Vap to 19200 
### set the baudrate of Pump to 19200
RS232_Header_Vap_port = serial.Serial(
    port=RS232_Header_Vap_port_path,
    baudrate=115200, 
    bytesize=8, 
    stopbits=1, 
    parity='N'
    )
lst_port.append(RS232_Header_Vap_port)
# Header Rreading, RTU func code 03
Header_Vap_RTU_R = Modbus.RTU(Header_Vap_id, '03', '008A', '0001')
Header_Vap_slave_R = Modbus.Slave(Header_Vap_id, Header_Vap_RTU_R.rtu)
print(Header_Vap_RTU_R.rtu)
# Header Writing, RTU func code 06
Header_Vap_SV = '0000'  
Header_Vap_RTU_W = Modbus.RTU(Header_Vap_id, '06', '0000', Header_Vap_SV)
Header_Vap_slave_W = Modbus.Slave(Header_Vap_id, Header_Vap_RTU_W.rtu)

print('Port setting: succeed')


def Header_Vap_collect(port, slave):
    try:
        #slave.time_readings.append(time.time()-start)
        port.write(bytes.fromhex(slave.rtu)) #hex to binary(byte) 

        time.sleep(1)

        print(port.inWaiting())
        #print(port.read(wait_data).hex())
        readings = port.read(port.inWaiting()).hex()
        print(readings)
        print(int(readings[-8:-4],16)/10)
        slave.lst_readings.append(readings)
        print('Header_Vap_collect done')
        port.reset_input_buffer() # reset the buffer after each reading process
    except:
        pass
while True:
    Header_Vap_collect(RS232_Header_Vap_port, Header_Vap_slave_R)