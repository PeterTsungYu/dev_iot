# %%
# python packages
#!/usr/bin/python
# -*- coding:utf-8 -*-

import time
from datetime import datetime
import RPi.GPIO as GPIO
import serial

# custome modules
import params
import config
import MQTT_config
import Mariadb_config
import Modbus
import ADDA_ADS1256
import ADDA_DAC8532

print('Import: succeed')

#-------------------------Start line-----------------------------------------
timeit = datetime.now().strftime('%Y_%m_%d_%H_%M')
print(f'Execution time is {timeit}')
print('=='*30)
start = time.time()

#-------------------------Open ports--------------------------------------
try:
    for device_port in config.lst_ports:
        if type(device_port.port) is serial.serialposix.Serial:
            if (not device_port.port.is_open):
                device_port.port.open() # code here........
            device_port.port.reset_input_buffer() #flush input buffer
            device_port.port.reset_output_buffer() #flush output buffer
    print('serial ports open')
    
    ADC = ADDA_ADS1256.ADS1256()
    DAC = ADDA_DAC8532.DAC8532()
    ADC.ADS1256_init()
    DAC.DAC8532_Out_Voltage(0x30, 3)
    DAC.DAC8532_Out_Voltage(0x34, 3)
    '''
    GPIO.setup(config.channel_DFM, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(config.channel_DFM_AOG, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    def DFM_data_collect(self):
        config.DFM_slave.time_readings.append(time.time())
    def DFM_AOG_data_collect(self):
        config.DFM_AOG_slave.time_readings.append(time.time())
    GPIO.add_event_detect(config.channel_DFM, GPIO.RISING, callback=DFM_data_collect)
    GPIO.add_event_detect(config.channel_DFM_AOG, GPIO.RISING, callback=DFM_AOG_data_collect)
    '''
    print('GPIO ports open')
    
except Exception as ex:
    print ("open serial port error: " + str(ex))
    for device_port in config.lst_ports:
        if type(device_port.port) is serial.serialposix.Serial: 
            device_port.port.close()
    exit() 

#-------------------------Sub-Threadingggg-----------------------------------------
try:
    for device_port in config.lst_ports: 
        device_port.serial_funcs(start)
        device_port.parallel_funcs(start) 
        print(device_port.thread_funcs)
        for subthread in device_port.thread_funcs:
            subthread.start()
            print('start', subthread.name)
           
except Exception as ex:
    print ("Threading funcs error: " + str(ex))
    for device_port in config.lst_ports:
        if type(device_port.port) is serial.serialposix.Serial:
            device_port.port.close()
    GPIO.cleanup()
    print ("\r\nProgram end     ")
    exit()

#-------------------------Main Threadingggg-----------------------------------------
try:
    while not params.kb_event.isSet():
        if not params.ticker.wait(params.sample_time):
            print("=="*10 + f'Elapsed time: {round((time.time()-start),2)}' + "=="*10)
            ADC_Value = ADC.ADS1256_GetAll()
            print ("0 ADC = %lf"%(ADC_Value[0]*5.0/0x7fffff))
            print ("1 ADC = %lf"%(ADC_Value[1]*5.0/0x7fffff))
            print ("2 ADC = %lf"%(ADC_Value[2]*5.0/0x7fffff))
            print ("3 ADC = %lf"%(ADC_Value[3]*5.0/0x7fffff))
            print ("4 ADC = %lf"%(ADC_Value[4]*5.0/0x7fffff))
            print ("5 ADC = %lf"%(ADC_Value[5]*5.0/0x7fffff))
            print ("6 ADC = %lf"%(ADC_Value[6]*5.0/0x7fffff))
            print ("7 ADC = %lf"%(ADC_Value[7]*5.0/0x7fffff))

            temp = (ADC_Value[0]>>7)*5.0/0xffff
            print ("DAC :",temp)
            print ("\33[10A")
            DAC.DAC8532_Out_Voltage(ADDA_DAC8532.channel_A, temp)
            DAC.DAC8532_Out_Voltage(ADDA_DAC8532.channel_B, 3.3 - temp)
        
except KeyboardInterrupt: 
    print(f"Keyboard Interrupt in main thread!")
    print("=="*30)
except Exception as ex:
    print ("Main threading error: " + str(ex))    
    print("=="*30)
finally:
    print("=="*30)
    for device_port in config.lst_ports: 
        if type(device_port.port) is serial.serialposix.Serial:
            device_port.port.close()
        elif device_port.port == 'GPIO':
            GPIO.cleanup()
        Modbus.logger.info(f'Close {device_port.name}, err are {device_port.err_values}')
        Modbus.logger.info(f'correct rates : {[f"{k}:{round((v[1]-v[0])/(v[1] + 0.00000000000000001)*100,2)}%" for k,v in device_port.err_values.items()]}')
    Modbus.logger.info(f"Program duration: {time.time() - start}")
    
    #if Mariadb_config.get(conn):
        #Mariadb_config.conn.close()

    print("close connection to MariaDB")
    MQTT_config.client_0.loop_stop()
    MQTT_config.client_0.disconnect()
    print("close connection to MQTT broker")
    print ("close GPIO")
    GPIO.cleanup()
    print('kill main thread')
    exit()