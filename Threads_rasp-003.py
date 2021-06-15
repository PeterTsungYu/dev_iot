# %%
# python packages
import RPi.GPIO as GPIO
import numpy as np
import time
from datetime import datetime
import threading
import re

# custome modules
import Modbus
import config
import MQTT_config

print('Import: succeed')


#-----port and slave setting----------------------------------------------------------------
lst_port = []
lst_port.append(config.RS485_TCHeader_1_port)

# TCHeader Rreading, RTU func code 03, PV value site at '008A', data_len is 1 ('0001')
TCHeader_1_RTU_R = Modbus.RTU(config.TCHeader_1_id, '03', '008A', '0001')
TCHeader_1_slave = Modbus.Slave(config.TCHeader_1_id, TCHeader_1_RTU_R.rtu,) # list[0]:read, list[1]:write

#print(TCHeader_1_slave.rtu)
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
## RS485
def RS485_data_collect(port):
    TCHeader_0_count_err = [0,0] # [collect_err, set_err]
    TCHeader_1_count_err = [0,0] # [collect_err, set_err]
    while not config.kb_event.isSet():
        TCHeader_count_err = Modbus.TCHeader_comm(
            start, port, TCHeader_1_slave, 7, TCHeader_1_count_err, 
            MQTT_config.sub_SV1_event,
            MQTT_config.sub_SV1,
            ) # wait for 7 bytes
        #Modbus.MFC_data_collect(start, port, MFC_slave, 49)
    port.close()
    print('kill TCHeader_comm')
    print(f'Final TCHeader_comm: {TCHeader_count_err} errors occured')
    #print('kill MFC_data_collect')
    #Modbus.barrier_kill.wait()

RS485_data_collect = threading.Thread(
    target=RS485_data_collect, 
    args=(config.RS485_TCHeader_1_port,)
    )
TCHeader_1_analyze = threading.Thread(
    target=Modbus.TCHeader_analyze, 
    args=(start, TCHeader_1_slave, RPi_Server,),
    )
'''
MFC_data_analyze = threading.Thread(
    target=Modbus.MFC_data_analyze, 
    args=(start, MFC_slave, RPi_Server,),
    )
'''
lst_thread.append(RS485_data_collect)
lst_thread.append(TCHeader_1_analyze)
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
    print(TCHeader_1_slave.readings)
    exit()

