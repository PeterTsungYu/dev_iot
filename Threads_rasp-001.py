# %%
# python packages
import time
from datetime import datetime
import RPi.GPIO as GPIO
import pigpio
import serial
import multiprocessing

# custome modules
import params
import config
import MQTT_config
import Mariadb_config
import Modbus

print('Import: succeed')

#-------------------------Start line-----------------------------------------
timeit = datetime.now().strftime('%Y_%m_%d_%H_%M')
print(f'Execution time is {timeit}')
print('=='*30)
start = time.time()
PIG = pigpio.pi()

#-------------------------Open ports--------------------------------------
try:
    for device_port in config.lst_ports:
        if type(device_port.port) is serial.serialposix.Serial:
            if (not device_port.port.is_open):
                device_port.port.open() # code here........
            device_port.port.reset_input_buffer() #flush input buffer
            device_port.port.reset_output_buffer() #flush output buffer
    print('serial ports open')
    
    PIG.set_mode(config.channel_DFM, pigpio.INPUT)
    PIG.set_pull_up_down(config.channel_DFM, pigpio.PUD_DOWN)
    PIG.set_mode(config.channel_DFM_AOG, pigpio.INPUT)
    PIG.set_pull_up_down(config.channel_DFM_AOG, pigpio.PUD_DOWN)
    def DFM_data_collect(user_gpio, level, tick):
        config.DFM_slave.lst_readings.put(time.time())
    def DFM_AOG_data_collect(user_gpio, level, tick):
        config.DFM_AOG_slave.lst_readings.put(time.time())
    PIG.callback(user_gpio=config.channel_DFM, edge=pigpio.RISING_EDGE, func=DFM_data_collect)
    PIG.callback(user_gpio=config.channel_DFM_AOG, edge=pigpio.RISING_EDGE, func=DFM_AOG_data_collect)
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
        for process in device_port.comm_funcs:
            process.start()
            print('start', process.name)
    params.sample_ticker.set()
    
except Exception as ex:
    print ("Threading funcs error: " + str(ex))
    for device_port in config.lst_ports:
        if type(device_port.port) is serial.serialposix.Serial:
            device_port.port.close()
    exit()

#-------------------------Main Threadingggg-----------------------------------------
try:
    while not params.kb_event.is_set():
        print("=="*10 + f'Elapsed time: {round((time.time()-start),2)}' + "=="*10)
        time.sleep(params.sample_time)
        t = time.time()
        params.sample_ticker.clear()
        
        for device_port in config.lst_ports: 
            device_port.parallel_funcs(start) 
            for func in device_port.analyze_funcs:
                func.start()
            for func in device_port.control_funcs:
                func.start()
    
        for device_port in config.lst_ports: 
            for func in device_port.analyze_funcs:
                func.join()
            for func in device_port.control_funcs:
                func.join()

        params.sample_ticker.set()
        print(f'calc sample time: {time.time()-t}')
        
except KeyboardInterrupt: 
    print(f"Keyboard Interrupt in main thread!")
    print("=="*30)
except Exception as ex:
    print ("Main threading error: " + str(ex))    
    print("=="*30)
finally:
    print("=="*30)
    print(multiprocessing.active_children())
    for process in multiprocessing.active_children():
        process.join()
    for device_port in config.lst_ports: 
        if type(device_port.port) is serial.serialposix.Serial:
            device_port.port.close()
        elif device_port.port == 'GPIO':
            GPIO.cleanup()
        Modbus.logger.info(f'Close {device_port.name}, err are {[f"{k}:{v[:]}" for k,v in device_port.err_values.items()]}')
        Modbus.logger.info(f'correct rates : {[f"{k}:{round((v[1] - v[0] + v[2])/(v[1] + 0.00000000000000001)*100,2)}%" for k,v in device_port.err_values.items()]}')
    Modbus.logger.info(f"Program duration: {time.time() - start}")
    Mariadb_config.conn.close()
    print("close connection to MariaDB")
    MQTT_config.client_0.loop_stop()
    MQTT_config.client_0.disconnect()
    print("close connection to MQTT broker")
    print('kill main thread')
    exit()
# %%
