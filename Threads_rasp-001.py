# %%
# python packages
import time
from datetime import datetime
import threading
import RPi.GPIO as GPIO

# custome modules
import Modbus
import config
import MQTT_config
import Mariadb_config

print('Import: succeed')

#-----port and slave setting----------------------------------------------------------------
lst_port = []
lst_port.append(config.RS485_port)
lst_port.append(config.RS232_port)
lst_port.append(config.Scale_port)
lst_port.append(config.Setup_port)

# ADAM_TC
# RTU func code 03, PV value site starts at '0000', data_len is 8 ('0008')
ADAM_TC_RTU = Modbus.RTU(config.ADAM_TC_id, '03', '0000', '0008')
ADAM_TC_slave = Modbus.Slave(name='ADAM_TC', 
                            port='RS485_port', 
                            idno=config.ADAM_TC_id, 
                            rtu=ADAM_TC_RTU.rtu
                            )

# GA slave
GA_RTU = '11 01 60 8E'
GA_slave = Modbus.Slave(name='GA', 
                        port='RS232_port', 
                        idno=config.GA_id, 
                        rtu=GA_RTU
                        )

# Scale slave
Scale_slave = Modbus.Slave(name='Scale', 
                            port='Scale_port', 
                            idno=config.Scale_id, 
                            rtu=''
                            )

# TCHeader Rreading, RTU func code 03, PV value site at '008A', data_len is 1 ('0001')
Header_EVA_RTU_R = Modbus.RTU(config.Header_EVA_id, '03', '008A', '0001')
Header_EVA_slave = Modbus.Slave(name='Header_EVA', 
                                port='Setup_port', 
                                idno=config.Header_EVA_id, 
                                rtu=Header_EVA_RTU_R.rtu
                                )

Header_BR_RTU_R = Modbus.RTU(config.Header_BR_id, '03', '008A', '0001')
Header_BR_slave = Modbus.Slave(name='Header_BR', 
                                port='Setup_port', 
                                idno=config.Header_BR_id, 
                                rtu=Header_BR_RTU_R.rtu
                                )

# ADAM_SET_slave, RTU func code 03, channel site at '0000-0003', data_len is 4 ('0004')
## ch00:+-10V, ch01:0-5V, ch02:0-5V, ch03:0-5V
ADAM_SET_RTU_R = Modbus.RTU(config.ADAM_SET_id, '03', '0000', '0004') # only ch0
ADAM_SET_slave = Modbus.Slave(name='ADAM_SET', 
                                port='Setup_port', 
                                idno=config.ADAM_SET_id, 
                                rtu=ADAM_SET_RTU_R.rtu
                                )

# ADAM_READ_slave, RTU func code 03, channel site at '0000-0008', data_len is 8 ('0008')
## ch00:4-20mA, ch01:0-5V, ch04:0-5V, ch05:0-5V, ch06:0-5V
ADAM_READ_RTU_R = Modbus.RTU(config.ADAM_READ_id, '03', '0000', '0008') # only ch0
ADAM_READ_slave = Modbus.Slave(name='ADAM_READ', 
                                port='Setup_port', 
                                idno=config.ADAM_READ_id, 
                                rtu=ADAM_READ_RTU_R.rtu
                                )

# DFMs' slaves
DFM_slave = Modbus.Slave(name='DFM', 
                        port='GPIO_port', 
                        idno=config.DFM_id, 
                        rtu=''
                        )
DFM_AOG_slave = Modbus.Slave(name='DFM_AOG', 
                        port='GPIO_port', 
                        idno=config.DFM_AOG_id, 
                        rtu=''
                        )

print('Port setting: succeed')

#-------------------------define Threads and Events-----------------------------------------
timeit = datetime.now().strftime('%Y_%m_%d_%H_%M')
print(f'Execution time is {timeit}')
print('=='*30)
start = time.time()

# RS485_port
ADAM_TC_collect = threading.Thread(
    target=Modbus.ADAM_TC_collect, 
    args=(start, config.RS485_port, ADAM_TC_slave, 21,),
    )
ADAM_TC_analyze = threading.Thread(
    target=Modbus.ADAM_TC_analyze, 
    args=(start, ADAM_TC_slave,),
    )

## RS232_port
RS232_data_collect = threading.Thread(
    target=Modbus.RS232_data_collect, 
    args=(config.RS232_port,)
    )
GA_data_analyze = threading.Thread(
    target=Modbus.GA_data_analyze, 
    args=(start, GA_slave,),
    )

# Scale_port
Scale_data_collect = threading.Thread(
    target=Modbus.Scale_data_collect, 
    args=(start, config.Scale_port, Scale_slave, 0,),
    )
Scale_data_analyze = threading.Thread(
    target=Modbus.Scale_data_analyze, 
    args=(start, Scale_slave, 'Scale',),
    )

# add READ
def Setup_data_collect(port):
    global Header_EVA_collect_err, Header_BR_collect_err, ADAM_SET_collect_err
    while not config.kb_event.isSet():
        Header_EVA_collect_err = Modbus.TCHeader_comm(
            start, port, Header_EVA_slave, 7, Header_EVA_collect_err, 
            MQTT_config.sub_Topics['Header_EVA_SV']['event'], #todo: config a general slave class
            MQTT_config.sub_Topics['Header_EVA_SV']['value'], 
            ) # wait for 7 bytes
        #print(Header_EVA_collect_err)
        Header_BR_collect_err = Modbus.TCHeader_comm(
            start, port, Header_BR_slave, 7, Header_BR_collect_err, 
            MQTT_config.sub_Topics['Header_BR_SV']['event'], #todo: config a general slave class
            MQTT_config.sub_Topics['Header_BR_SV']['value'],
            ) # wait for 7 bytes
        #print(Header_BR_collect_err)
        ADAM_SET_collect_err = Modbus.ADAM_SET_comm(
            start, port, ADAM_SET_slave, 7, ADAM_SET_collect_err, 
            MQTT_config.sub_Topics['ADAM_SET_SV0']['event'], #todo: config a general slave class
            MQTT_config.sub_Topics['ADAM_SET_SV0']['value'], 
            ) # wait for 7 bytes == 7 Hex numbers
    port.close()
    print('kill TCHeader_comm')
    print(f'Final TCHeader_comm: {Header_EVA_collect_err} and {Header_BR_collect_err} errors occured')
    print('kill ADAM_SET_comm')
    print(f'Final ADAM_SET_comm: {ADAM_SET_collect_err} errors occured')
    
Setup_data_collect = threading.Thread(
    target=Setup_data_collect, 
    args=(config.RS485_port,)
    )
Header_EVA_analyze = threading.Thread(
    target=Modbus.TCHeader_analyze, 
    args=(start, Header_EVA_slave, 'Header_EVA_PV',),
    )
Header_BR_analyze = threading.Thread(
    target=Modbus.TCHeader_analyze, 
    args=(start, Header_BR_slave, 'Header_BR_PV',),
    )
ADAM_SET_analyze = threading.Thread(
    target=Modbus.ADAM_SET_analyze, 
    args=(start, ADAM_SET_slave, 'ADAM_SET_PV0',),
    )
# add READ func

## GPIO_port
# Edge detection
def DFM_data_collect():
    DFM_slave.time_readings.append(time.time())
DFM_data_analyze = threading.Thread(
    target=Modbus.DFM_data_analyze, 
    args=(start, DFM_slave,),
    )

def DFM_AOG_data_collect():
    DFM_AOG_slave.time_readings.append(time.time())
DFM_AOG_data_analyze = threading.Thread(
    target=Modbus.DFM_AOG_data_analyze, 
    args=(start, DFM_AOG_slave,),
    )

lst_thread = []
lst_thread.append(ADAM_TC_collect)
lst_thread.append(ADAM_TC_analyze)
lst_thread.append(RS232_data_collect)
lst_thread.append(GA_data_analyze)
lst_thread.append(Scale_data_collect)
lst_thread.append(Scale_data_analyze)
lst_thread.append(Setup_data_collect)
lst_thread.append(Header_EVA_analyze)
lst_thread.append(Header_BR_analyze)
lst_thread.append(ADAM_SET_analyze)
lst_thread.append(DFM_data_analyze)
lst_thread.append(DFM_AOG_data_analyze)

#-------------------------Open ports--------------------------------------
try:
    for port in lst_port: 
        if not port.is_open:
            port.open() # code here........
        port.reset_input_buffer() #flush input buffer
        port.reset_output_buffer() #flush output buffer
    print('serial ports open')
    
    GPIO.setup(config.channel_DFM, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(config.channel_DFM_AOG, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    print('GPIO ports open')
    
except Exception as ex:
    print ("open serial port error: " + str(ex))
    for port in lst_port: 
        port.close()
    exit() 

#-------------------------Sub Threadingggg-----------------------------------------
for subthread in lst_thread:
    subthread.start()

GPIO.add_event_detect(config.channel_DFM, GPIO.RISING, callback=DFM_data_collect)
GPIO.add_event_detect(config.channel_DFM_AOG, GPIO.RISING, callback=DFM_AOG_data_collect)

#-------------------------Main Threadingggg-----------------------------------------
try:
    while not config.kb_event.isSet():
        if not config.ticker.wait(config.sample_time):
            print("=="*10 + f'Elapsed time: {round((time.time()-start),2)}' + "=="*10)
        
except KeyboardInterrupt: 
    print(f"Keyboard Interrupt in main thread!")
    print("=="*30)
except Exception as ex:
    print ("Main threading error: " + str(ex))    
    print("=="*30)
finally:
    print("=="*30)
    GPIO.cleanup()
    print(f"Program duration: {time.time() - start}")
    Mariadb_config.conn.close()
    print("close connection to MariaDB")
    MQTT_config.client_0.loop_stop()
    MQTT_config.client_0.disconnect()
    print("close connection to MQTT broker")
    print('kill main thread')
    exit()