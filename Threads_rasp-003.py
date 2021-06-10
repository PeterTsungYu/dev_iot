# %%
import serial
import RPi.GPIO as GPIO
import numpy as np
import time
from datetime import datetime
import Modbus
import threading
import re
import config

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
# Header Rreading, RTU func code 03, PV value site at '008A', data_len is 1 ('0001')
Header_Vap_RTU_R = Modbus.RTU(Header_Vap_id, '03', '008A', '0001')
# Header Writing, RTU func code 06, SV value site at '0000'
Header_Vap_SV = '0000'  # setting Header value in hex
Header_Vap_RTU_W = Modbus.RTU(Header_Vap_id, '06', '0000', Header_Vap_SV)
Header_Vap_slave = Modbus.Slave(Header_Vap_id, [Header_Vap_RTU_R.rtu, Header_Vap_RTU_W.rtu]) # list[0]:read, list[1]:write

#print(Header_Vap_slave.rtu)
print('Port setting: succeed')

#-----RPi Server_DB setting----------------------------------------------------------------
RPi_Server = Modbus.Slave()
# initiate the server data sites
RPi_Server.readings = [0] * 19 # 19 data entries

#-------------------------define Threads and Events-----------------------------------------
timeit = datetime.now().strftime('%Y_%m_%d_%H_%M')
print(f'Execution time is {timeit}')
print('=='*30)
start = time.time()

lst_thread = []
## RS232
def RS232_data_collect(port):
    count_err = 0
    while not config.kb_event.isSet():
        count_err = Modbus.TCHeader_comm(start, port, Header_Vap_slave, 7, count_err) # wait for 7 bytes
        #Modbus.MFC_data_collect(start, port, MFC_slave, 49)
    port.close()
    print('kill TCHeader_comm')
    print(f'Final TCHeader_comm: {count_err} errors occured')
    #print('kill MFC_data_collect')
    #Modbus.barrier_kill.wait()

RS232_data_collect = threading.Thread(
    target=RS232_data_collect, 
    args=(RS232_Header_Vap_port,)
    )
Header_Vap_analyze = threading.Thread(
    target=Modbus.TCHeader_analyze, 
    args=(start, Header_Vap_slave, RPi_Server,),
    )
'''
MFC_data_analyze = threading.Thread(
    target=Modbus.MFC_data_analyze, 
    args=(start, MFC_slave, RPi_Server,),
    )
'''
lst_thread.append(RS232_data_collect)
lst_thread.append(Header_Vap_analyze)
#lst_thread.append(MFC_data_analyze)

#-------------------------Open ports--------------------------------------
try:
    for port in lst_port: 
        if not port.is_open:
            port.open() # code here........
        port.reset_input_buffer() #flush input buffer
        port.reset_output_buffer() #flush output buffer
    print('serial ports open')
    
    #GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    #GPIO.setup(channel_DFM, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    #print('GPIO ports open')
    
except Exception as ex:
    print ("open serial port error: " + str(ex))
    for port in lst_port: 
        port.close()
    exit() 

#-------------------------Sub Threadingggg-----------------------------------------
for subthread in lst_thread:
    subthread.start()

#GPIO.add_event_detect(channel_DFM, GPIO.RISING, callback=DFM_data_collect)

#-------------------------Main Threadingggg-----------------------------------------
try:
    while not config.kb_event.isSet():
        if not config.ticker.wait(config.sample_time):
        #Modbus.barrier_analyze.wait()
            print("=="*10 + f'Elapsed time: {round((time.time()-start),2)}' + "=="*10)
        #Modbus.barrier_cast.wait()
            #print("=="*10 + f'Casting done. Elapsed time: {time.time()-start}' + "=="*10)
        
except KeyboardInterrupt: 
    print(f"Keyboard Interrupt in main thread!")
    print("=="*30)
except Exception as ex:
    print ("Main threading error: " + str(ex))    
    print("=="*30)
finally:
    #Modbus.barrier_kill.wait()
    print("=="*30)
    #GPIO.cleanup()
    print(f"Program duration: {time.time() - start}")
    print('kill main thread')
    print(Header_Vap_slave.readings)
    exit()

