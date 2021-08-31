# %%
# python packages
import time
from datetime import datetime
import RPi.GPIO as GPIO

# custome modules
import params
import config
import MQTT_config
import Mariadb_config

print('Import: succeed')

#-------------------------Start line-----------------------------------------
timeit = datetime.now().strftime('%Y_%m_%d_%H_%M')
print(f'Execution time is {timeit}')
print('=='*30)
start = time.time()

#-------------------------Open ports--------------------------------------
lst_ports = [config.RS485_port, config.Scale_port, config.RS232_port, config.Setup_port]

try:
    for device_port in lst_ports: 
        if not device_port.port.is_open:
            device_port.port.open() # code here........
        device_port.port.reset_input_buffer() #flush input buffer
        device_port.port.reset_output_buffer() #flush output buffer
    print('serial ports open')
    
    GPIO.setup(config.channel_DFM, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(config.channel_DFM_AOG, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    print('GPIO ports open')
    
except Exception as ex:
    print ("open serial port error: " + str(ex))
    for device_port in lst_ports: 
        device_port.port.close()
    exit() 

#-------------------------Sub-Threadingggg-----------------------------------------
try:
    for device_port in lst_ports: 
        device_port.serial_funcs(start)
        device_port.parallel_funcs(start) 
        for subthread in device_port.thread_funcs:
            subthread.start()
    print('Threading funcs')
           
except Exception as ex:
    print ("Threading funcs error: " + str(ex))
    for device_port in lst_ports:
            device_port.port.close()
    exit() 

#-------------------------GPIO Threadingggg-----------------------------------------
def DFM_data_collect():
    config.DFM_slave.time_readings.append(time.time())
def DFM_AOG_data_collect():
    config.DFM_AOG_slave.time_readings.append(time.time())
GPIO.add_event_detect(config.channel_DFM, GPIO.RISING, callback=DFM_data_collect)
GPIO.add_event_detect(config.channel_DFM_AOG, GPIO.RISING, callback=DFM_AOG_data_collect)
config.GPIO_port.serial_funcs(start)
config.GPIO_port.parallel_funcs(start)
for subthread in config.GPIO_port.thread_funcs:
    subthread.start()

#-------------------------Main Threadingggg-----------------------------------------
try:
    while not params.kb_event.isSet():
        if not params.ticker.wait(params.sample_time):
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