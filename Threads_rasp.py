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

    Modbus.PIG.set_mode(config.channel_Relay01_IN1, Modbus.pigpio.OUTPUT)
    Modbus.PIG.write(config.channel_Relay01_IN1, 1)
    Modbus.PIG.set_mode(config.channel_Relay01_IN2, Modbus.pigpio.OUTPUT)
    Modbus.PIG.write(config.channel_Relay01_IN2, 1)
    Modbus.PIG.set_mode(config.GPIO_TTL, Modbus.pigpio.OUTPUT)
    Modbus.PIG.write(config.GPIO_TTL, 1)
    Modbus.PIG.set_mode(config.GPIO_MOS, Modbus.pigpio.OUTPUT)
    Modbus.PIG.write(config.GPIO_MOS, 0)
    
    PIG.set_mode(config.channel_DFM, pigpio.INPUT)
    PIG.set_mode(config.channel_DFM_AOG, pigpio.INPUT)

    def DFM_data_collect(user_gpio, level, tick):
        print('DFM')
        config.DFM_slave.time_readings.put(time.time())
    def DFM_AOG_data_collect(user_gpio, level, tick):
        print('DFM_AOG')
        config.DFM_AOG_slave.time_readings.put(time.time())
    PIG.callback(user_gpio=config.channel_DFM, edge=pigpio.RISING_EDGE, func=DFM_data_collect)
    PIG.callback(user_gpio=config.channel_DFM_AOG, edge=pigpio.RISING_EDGE, func=DFM_AOG_data_collect)
    print('GPIO ports open')
    
except Exception as ex:
    print("open serial port error: " + str(ex))
    for device_port in config.lst_ports:
        if type(device_port.port) is serial.serialposix.Serial: 
            device_port.port.close()
    exit() 

#-------------------------Sub-Threadingggg-----------------------------------------
try:
    for device_port in config.lst_ports: 
        device_port.comm_funcs(start)
        # device_port.analyze_funcs(start)
        device_port.control_funcs(start)
    
    MQTT_config.multi_pub_process.start()
    Mariadb_config.multi_insert_process.start()

except Exception as ex:
    print("Threading funcs error: " + str(ex))
    for device_port in config.lst_ports:
        if type(device_port.port) is serial.serialposix.Serial:
            device_port.port.close()
        elif device_port.port == 'GPIO':
            for slave in device_port.slaves:
                Modbus.PIG.write(slave.id, 0)
            Modbus.PIG.stop()
            print ("close GPIO")
    print ("\r\nProgram end")
    exit()

#-------------------------Main Threadingggg-----------------------------------------
try:
    while not params.kb_event.is_set():
        print("=="*10 + f'Elapsed time: {round((time.time()-start),2)}' + "=="*10)
        time.sleep(1)
    
    print([i.name for i in multiprocessing.active_children()])
    active_managers = [i for i in multiprocessing.active_children() if 'SyncManager' in i.name]
    for process in multiprocessing.active_children():
        if process not in active_managers:
            print(f'Terminate process: {process.name}')
            process.terminate()
    for process in multiprocessing.active_children():
        if process not in active_managers:
            print(f'Join process: {process.name}')
            process.join()
    for manager in active_managers:
        print(f'Terminate process: {manager.name}')
        manager.terminate()
    for manager in active_managers:
        print(f'Join process: {manager.name}')
        manager.join()
    
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
            for slave in device_port.slaves:
                if isinstance(slave.id, int): 
                    Modbus.PIG.write(slave.id, 0)
            Modbus.PIG.stop()
            print ("close GPIO")
            
        Modbus.logger.critical(f'Close {device_port.name}, err are {[f"{k}:{v[:]}" for k,v in device_port.err_values.items()]}')
        Modbus.logger.critical(f'correct rates : {[f"{k}:{round(v[2]/(v[1] + 0.00000000000000001)*100,2)}%" for k,v in device_port.err_values.items()]}')
        Modbus.logger.critical(f'Close {device_port.name}, recur are {[f"{k}:{v[:]}" for k,v in device_port.recur_count.items()]}')
    Modbus.logger.critical(f"Program duration: {time.time() - start}")
    print('kill main thread')
    exit()
# %%
