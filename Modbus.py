# pip3 install crccheck
# pip3 install numpy 
## for numpy, also do this: sudo apt-get install libatlas-base-dev
# pip3 install pyserial

#python packages
from crccheck.crc import Crc16Modbus
import numpy as np
import serial
import time
import re
import random

#custom modules
import config
import MQTT_config

#-------------------------RTU & Slave--------------------------------------
class RTU: # generate the CRC for the complete RTU 
    def __init__(self, idno='', func_code='', data_site='', data_len=''):
        self.id = idno # id number of slave
        self.data_len = data_len
        data_struc = idno + func_code + data_site + data_len
        crc = Crc16Modbus.calchex(bytearray.fromhex(data_struc))
        self.rtu = data_struc + crc[-2:] + crc[:2]


class Slave: # Create Slave data store 
    def __init__(self, idno='', rtu=''):
        self.id = idno # id number of slave
        self.rtu = rtu # str or a list. list[0]:read, list[1]:write 
        self.lst_readings = [] # record readings
        self.time_readings = 0 # record time
        self.readings = [] # for all data 

#------------------------------Port func---------------------------------
def RS232_data_collect(start, port, slave, *funcs):
    while not config.kb_event.isSet():
        for func in funcs:
            func(start, port, slave, 31, collect_err)
    port.close()
    print('kill GA_data_collect')
    print(f'Final GA_data_collect: {collect_err} errors occured')


#------------------------------Collect and Analyze func---------------------------------
def tohex(value):
    value = int(value)
    hex_value = hex(value)[2:]
    add_zeros = 4 - len(hex_value)
    hex_value = add_zeros * '0' + hex_value
    return hex_value


def ADAM_TC_collect(start, port, slave, wait_data, collect_err):
    #start = time.time()
    while not config.kb_event.isSet():
        try:
            slave.time_readings.append(round(time.time()-start, 2))
            port.write(bytes.fromhex(slave.rtu)) #hex to binary(byte) 

            time.sleep(config.time_out)

            # look up the buffer for 21 bytes, which is for 8 channels data length
            if port.inWaiting() >= wait_data: 
                readings = port.read(wait_data).hex() # after reading, the buffer will be clean
                crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
                # check sta, func code, datalen, crc
                if (readings[0:2] == slave.id) and (readings[2:4] == '03') and ((crc[-2:] + crc[:2]) == readings[-4:]):
                    slave.lst_readings.append(readings)
                    print(f'ADAM_TC_collect done: read from slave_{slave.id}')
                else:
                    port.reset_input_buffer() # reset the buffer if no read
                    MQTT_config.pub_Topics[collect_err] += 1
                    print('XX'*10 + f"ADAM_TC_collect error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[collect_err]} at {round((time.time()-start),2)}s: crc validation failed" + 'XX'*10) 
            else: # if data len is less than the wait data
                port.reset_input_buffer() # reset the buffer if no read
                MQTT_config.pub_Topics[collect_err] += 1
                print('XX'*10 + f"ADAM_TC_collect error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[collect_err]} at {round((time.time()-start),2)}s: data len is less than the wait data" + 'XX'*10) 
        except Exception as e:
            MQTT_config.pub_Topics[collect_err] += 1
            print('XX'*10 + f" {MQTT_config.pub_Topics[collect_err]} ADAM_TC_collect error at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
        finally:
            pass

    port.close()
    print('kill ADAM_TC_collect')
    print(f'Final ADAM_TC_collect: {MQTT_config.pub_Topics[collect_err]} errors occured')
    #barrier_kill.wait()


def ADAM_READ_collect(start, port, slave, wait_data, collect_err):
    #start = time.time()
    while not config.kb_event.isSet():
        try:
            slave.time_readings.append(round(time.time()-start, 2))
            port.write(bytes.fromhex(slave.rtu)) #hex to binary(byte) 

            time.sleep(config.time_out)

            # look up the buffer for 21 bytes, which is for 8 channels data length
            if port.inWaiting() >= wait_data: 
                readings = port.read(wait_data).hex() # after reading, the buffer will be clean
                crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
                # check sta, func code, datalen, crc
                if (readings[0:2] == slave.id) and (readings[2:4] == '03') and ((crc[-2:] + crc[:2]) == readings[-4:]):
                    slave.lst_readings.append(readings)
                    print(f'ADAM_READ_collect done: read from slave_{slave.id}')
                else:
                    port.reset_input_buffer() # reset the buffer if no read
                    MQTT_config.pub_Topics[collect_err] += 1
                    print('XX'*10 + f"ADAM_READ_collect error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[collect_err]} at {round((time.time()-start),2)}s: crc validation failed" + 'XX'*10) 
            else: # if data len is less than the wait data
                port.reset_input_buffer() # reset the buffer if no read
                MQTT_config.pub_Topics[collect_err] += 1
                print('XX'*10 + f"ADAM_READ_collect error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[collect_err]} at {round((time.time()-start),2)}s: data len is less than the wait data" + 'XX'*10) 
        except Exception as e:
            MQTT_config.pub_Topics[collect_err] += 1
            print('XX'*10 + f" {MQTT_config.pub_Topics[collect_err]} ADAM_READ_collect error at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
        finally:
            pass

    port.close()
    print('kill ADAM_READ_collect')
    print(f'Final ADAM_READ_collect: {MQTT_config.pub_Topics[collect_err]} errors occured')
    #barrier_kill.wait()


def Scale_data_collect(start, port, slave, wait_data, collect_err):
    #start = time.time()
    while not config.kb_event.isSet():
        try:
            slave.time_readings = time.time()-start
            time.sleep(config.time_out) # wait for the data input to the buffer
            if port.inWaiting() > wait_data:
                readings = port.read(port.inWaiting()).decode('utf-8')
                readings = [float(s) if s[0] != '-' else -float(s[1:]) for s in re.findall(r'[ \-][ .\d]{7}', readings)]
                slave.lst_readings.append(readings)
                print(f'Scale_data_collect done: read from slave_{slave.id}')
                port.reset_input_buffer() # reset the buffer after each reading process
            else: # if data len is no data
                MQTT_config.pub_Topics[collect_err] += 1
                print('XX'*10 + f"Scale_data_collect error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[collect_err]} at {round((time.time()-start),2)}s: data len is no data" + 'XX'*10)
        except Exception as e:
            MQTT_config.pub_Topics[collect_err] += 1
            print('XX'*10 + f"Scale_data_collect error: err_{MQTT_config.pub_Topics[collect_err]} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
        finally:
            pass

    port.close()
    print('kill Scale_data_collect')
    print(f'Final Scale_data_collect: {MQTT_config.pub_Topics[collect_err]} errors occured')
    #barrier_kill.wait()


def GA_data_collect(start, port, slave, wait_data, collect_err):
    #start = time.time()
    #while not config.kb_event.isSet(): # it is written in the ReadingThreads.py
    try:
        slave.time_readings.append(time.time()-start)
        port.write(bytes.fromhex(slave.rtu)) #hex to binary(byte) 

        time.sleep(config.time_out)

        #print(port.inWaiting())
        if port.inWaiting() >= wait_data: 
            readings = port.read(wait_data).hex() # after reading, the buffer will be clean
            crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
            # check sta, func code, datalen, crc
            if (readings[0:2] == slave.id) and (readings[2:4] == '03') and ((crc[-2:] + crc[:2]) == readings[-4:]):
                slave.lst_readings.append(readings)
                print(f'ADAM_READ_collect done: read from slave_{slave.id}')
            else:
                port.reset_input_buffer() # reset the buffer if no read
                MQTT_config.pub_Topics[collect_err] += 1
                print('XX'*10 + f"ADAM_READ_collect error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[collect_err]} at {round((time.time()-start),2)}s: crc validation failed" + 'XX'*10) 
        else: # if data len is less than the wait data
            port.reset_input_buffer() # reset the buffer if no read
            MQTT_config.pub_Topics[collect_err] += 1
            print('XX'*10 + f"ADAM_READ_collect error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[collect_err]} at {round((time.time()-start),2)}s: data len is less than the wait data" + 'XX'*10) 
    except Exception as e:
        MQTT_config.pub_Topics[collect_err] += 1
        print('XX'*10 + f" {MQTT_config.pub_Topics[collect_err]} GA_data_collect error at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
    finally:
        pass
    #port.close()
    #print('kill GA_data_collect')
    #print(f'Final GA_data_collect: {collect_err} errors occured')
    ##barrier_kill.wait()


def TCHeader_comm(start, port, slave, wait_data, collect_err, set_err, write_event, write_value):
    #start = time.time()
    #while not config.kb_event.isSet(): # it is written in the ReadingThreads.py
    if not write_event.isSet():
        try: # try to collect
            slave.time_readings = time.time()-start
            port.write(bytes.fromhex(slave.rtu)) #hex to binary(byte) 

            time.sleep(config.time_out)

            #print(port.inWaiting())
            #print(port.read(wait_data).hex())
            if port.inWaiting() >= wait_data: 
                readings = port.read(wait_data).hex() # after reading, the buffer will be clean
                crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
                #print(readings)
                #print(crc)
                # check sta, func code, datalen, crc
                if (readings[0:2] == slave.id) and (readings[2:4] == '03') and ((crc[-2:] + crc[:2]) == readings[-4:]):
                    slave.lst_readings.append(readings)
                    print(f'TCHeader_collect done: read from slave_{slave.id}')
                else:
                    port.reset_input_buffer() # reset the buffer if no read
                    MQTT_config.pub_Topics[collect_err] += 1
                    print('XX'*10 + f"TCHeader_collect error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[collect_err]} at {round((time.time()-start),2)}s: crc validation failed" + 'XX'*10) 
            else: # if data len is less than the wait data
                port.reset_input_buffer() # reset the buffer if no read
                MQTT_config.pub_Topics[collect_err] += 1
                print('XX'*10 + f"TCHeader_collect error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[collect_err]} at {round((time.time()-start),2)}s: data len is less than the wait data" + 'XX'*10) 
        except Exception as e:
            MQTT_config.pub_Topics[collect_err] += 1
            print('XX'*10 + f"TCHeader_collect error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[collect_err]} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
        finally:
            pass

    else:
        try: # try to set value
            #print(write_value)
            # TCHeader Writing, RTU func code 06, SV value site at '0000'
            TCHeader_SV = tohex(write_value*10)  # setting TCHeader value in hex
            #print(TCHeader_SV)
            TCHeader_RTU_W = RTU(slave.id, '06', '0000', TCHeader_SV) #md: subscription value and rtu 
            #print(TCHeader_RTU_W.rtu)

            port.write(bytes.fromhex(TCHeader_RTU_W.rtu)) #hex to binary(byte) #md: subscription value and rtu
            time.sleep(config.time_out)
            if port.inWaiting() >= 8: 
                readings = port.read(8).hex() # after reading, the buffer will be clean
                #print(readings)
                crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
                # check sta, func code, datalen, crc
                if (readings[0:2] == slave.id) and (readings[2:4] == '06') and ((crc[-2:] + crc[:2]) == readings[-4:]):
                    print(f'TCHeader_set done: write {write_value} to slave_{slave.id}')
                else:
                    port.reset_input_buffer() # reset the buffer if no read
                    MQTT_config.pub_Topics[set_err] += 1
                    print('XX'*10 + f"TCHeader_set error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[set_err]} at {round((time.time()-start),2)}s: crc validation failed" + 'XX'*10) 
            else: # if data len is less than the wait data
                port.reset_input_buffer() # reset the buffer if no read
                MQTT_config.pub_Topics[set_err] += 1
                print('XX'*10 + f"TCHeader_set error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[set_err]} at {round((time.time()-start),2)}s: data len is less than the wait data" + 'XX'*10) 
        except Exception as e:
            MQTT_config.pub_Topics[set_err] += 1
            print('XX'*10 + f"TCHeader_set error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[set_err]} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
        finally:
            write_event.clear()



def ADAM_SET_comm(start, port, slave, wait_data, collect_err, set_err, write_event, write_value):
    #start = time.time()
    #while not config.kb_event.isSet(): # it is written in the ReadingThreads.py
    if not write_event.isSet():
        try: # try to collect
            slave.time_readings = time.time()-start
            port.write(bytes.fromhex(slave.rtu)) #hex to binary(byte) 

            time.sleep(config.time_out)

            #print(port.inWaiting())
            #print(port.read(wait_data).hex())
            if port.inWaiting() >= wait_data: 
                readings = port.read(wait_data).hex() # after reading, the buffer will be clean
                crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
                #print(readings)
                #print(crc)
                # check sta, func code, datalen, crc
                if (readings[0:2] == slave.id) and (readings[2:4] == '03') and ((crc[-2:] + crc[:2]) == readings[-4:]):
                    slave.lst_readings.append(readings)
                    print(f'ADAM_collect done: read from slave_{slave.id}')
                else:
                    port.reset_input_buffer() # reset the buffer if no read
                    MQTT_config.pub_Topics[collect_err] += 1
                    print('XX'*10 + f"ADAM_collect error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[collect_err]} at {round((time.time()-start),2)}s: crc validation failed" + 'XX'*10) 
            else: # if data len is less than the wait data
                port.reset_input_buffer() # reset the buffer if no read
                MQTT_config.pub_Topics[collect_err] += 1
                print('XX'*10 + f"ADAM_collect error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[collect_err]} at {round((time.time()-start),2)}s: data len is less than the wait data" + 'XX'*10) 
        except Exception as e:
            MQTT_config.pub_Topics[collect_err] += 1
            print('XX'*10 + f"ADAM_collect error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[collect_err]} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
        finally:
            pass

    else:
        try: # try to set value
            #print(write_value)
            # TCHeader Writing, RTU func code 06, SV value site at '0000'
            _SV = tohex(write_value*10)  # setting TCHeader value in hex
            #print(TCHeader_SV)
            _RTU_W = RTU(slave.id, '06', '0000', _SV) #md: subscription value and rtu 
            #print(TCHeader_RTU_W.rtu)

            port.write(bytes.fromhex(_RTU_W.rtu)) #hex to binary(byte) #md: subscription value and rtu
            time.sleep(config.time_out)
            if port.inWaiting() >= 8: 
                readings = port.read(8).hex() # after reading, the buffer will be clean
                #print(readings)
                crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
                # check sta, func code, datalen, crc
                if (readings[0:2] == slave.id) and (readings[2:4] == '06') and ((crc[-2:] + crc[:2]) == readings[-4:]):
                    print(f'ADAM_set done: write {write_value} to slave_{slave.id}')
                else:
                    port.reset_input_buffer() # reset the buffer if no read
                    MQTT_config.pub_Topics[set_err] += 1
                    print('XX'*10 + f"ADAM_set error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[set_err]} at {round((time.time()-start),2)}s: crc validation failed" + 'XX'*10) 
            else: # if data len is less than the wait data
                port.reset_input_buffer() # reset the buffer if no read
                MQTT_config.pub_Topics[set_err] += 1
                print('XX'*10 + f"ADAM_set error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[set_err]} at {round((time.time()-start),2)}s: data len is less than the wait data" + 'XX'*10) 
        except Exception as e:
            MQTT_config.pub_Topics[set_err] += 1
            print('XX'*10 + f"ADAM_set error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[set_err]} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
        finally:
            write_event.clear()


def ADAM_TC_analyze(start, slave, pub_Topic, analyze_err):
    while not config.kb_event.isSet():
        if not config.ticker.wait(config.sample_time):
            lst_readings = slave.lst_readings
            time_readings = slave.time_readings
            #print(f'ADAM_TC_analyze: {lst_readings}')
            slave.lst_readings = []
            slave.time_readings = []
            try:
                arr_readings = np.array([[int(reading[i-4:i],16) for i in range(10,len(reading)-2,4)] for reading in lst_readings])
                lst_readings = tuple(np.round(1370/65535*(np.sum(arr_readings, axis=0) / len(lst_readings)), 1))
                #print(lst_readings)
                readings = tuple([round(time_readings[-1],2)]) + lst_readings
                #print(readings)

                # casting
                MQTT_config.pub_Topics[pub_Topic] = readings[-1]
                ## to slave data list
                slave.readings.append(readings)
                print(f'ADAM_TC_analyze done: record {readings} from slave_{slave.id}')

            except Exception as e:
                MQTT_config.pub_Topics[analyze_err] += 1
                print('XX'*10 + f" {MQTT_config.pub_Topics[analyze_err]} ADAM_TC_analyze error at {round((time.time()-start),2)}s: " + str(e) + 'XX'*5)
            finally:
                pass
                    
    print('kill ADAM_TC_analyze')
    print(f'Final ADAM_TC_analyze: {MQTT_config.pub_Topics[analyze_err]} errors occured')
    #barrier_kill.wait()

# revise
def ADAM_READ_analyze(start, slave, pub_Topic):
    collect_err = 0
    while not config.kb_event.isSet():
        if not config.ticker.wait(config.sample_time):
            lst_readings = slave.lst_readings
            time_readings = slave.time_readings
            #print(f'ADAM_TC_analyze: {lst_readings}')
            slave.lst_readings = []
            slave.time_readings = []
            try:
                arr_readings = np.array([[int(reading[i-4:i],16) for i in range(10,len(reading)-2,4)] for reading in lst_readings])
                lst_readings = tuple(np.round(1370/65535*(np.sum(arr_readings, axis=0) / len(lst_readings)), 1))
                #print(lst_readings)
                readings = tuple([round(time_readings[-1],2)]) + lst_readings
                #print(readings)

                # casting
                MQTT_config.pub_Topics[pub_Topic] = readings[-1]
                ## to slave data list
                slave.readings.append(readings)
                print(f'ADAM_READ_analyze done: record {readings} from slave_{slave.id}')

            except Exception as e:
                collect_err += 1
                print('XX'*10 + f" {collect_err} ADAM_READ_analyze error at {round((time.time()-start),2)}s: " + str(e) + 'XX'*5)
            finally:
                pass
                    
    print('kill ADAM_READ_analyze')
    print(f'Final ADAM_READ_analyze: {collect_err} errors occured')
    #barrier_kill.wait()


def DFM_data_analyze(start, slave, pub_Topic, analyze_err):
    #start = time.time()
    while not config.kb_event.isSet():
        if not config.ticker.wait(config.sample_time_DFM): # for each sample_time, collect data
            sampling_time = round(time.time()-start, 2)
            try: 
                time_readings = slave.time_readings
                slave.time_readings = []
                average_interval_lst = []
                # calc average min flow rate by each interval 
                for interval in range(30, 55, 5):
                    flow_rate_interval_lst = []
                    # for each interval, calculate the average flow rate
                    for i in range(interval, len(time_readings), interval):
                        # flow rate in [liter/s]
                        # 0.1 liter / pulse
                        flow_rate = 60 * 0.1 * (interval-1) / (time_readings[i-1] - time_readings[i-interval])
                        flow_rate_interval_lst.append(round(flow_rate, 2)) 
                    average_flow_rate_interval = round(sum(flow_rate_interval_lst) / len(flow_rate_interval_lst), 2)          
                    average_interval_lst.append(average_flow_rate_interval)
                    _average = round(sum(average_interval_lst) / len(average_interval_lst), 1)
                readings = tuple([sampling_time, _average])
                
                # casting
                MQTT_config.pub_Topics[pub_Topic] = readings[-1]
                ## to slave data list
                slave.readings.append(readings)
                print(f'DFM_data_analyze done: record {readings} from slave_{slave.id}')

            except Exception as e:
                MQTT_config.pub_Topics[analyze_err] += 1
                print('XX'*10 + f" {MQTT_config.pub_Topics[analyze_err]} DFM_data_analyze error at {round((time.time()-start),2)}s: " + str(e) + 'XX'*5)
            finally:
                pass

    print('kill DFM_data_analyze')
    print(f'Final DFM_data_analyze: {MQTT_config.pub_Topics[analyze_err]} errors occured')
    #barrier_kill.wait()


def DFM_AOG_data_analyze(start, slave, pub_Topic, analyze_err):
    #start = time.time()
    while not config.kb_event.isSet():
        if not config.ticker.wait(config.sample_time_DFM): # for each sample_time, collect data
            sampling_time = round(time.time()-start, 2)
            try: 
                time_readings = slave.time_readings
                slave.time_readings = []
                average_interval_lst = []
                # calc average min flow rate by each interval 
                for interval in range(5, 30, 5):
                #for interval in range(30, 55, 5):
                    flow_rate_interval_lst = []
                    # for each interval, calculate the average flow rate
                    for i in range(interval, len(time_readings), interval):
                        # flow rate in [liter/s]
                        # 0.01 liter / pulse
                        flow_rate = 60 * 0.01 * (interval-1) / (time_readings[i-1] - time_readings[i-interval])
                        flow_rate_interval_lst.append(round(flow_rate, 2)) 
                    average_flow_rate_interval = round(sum(flow_rate_interval_lst) / len(flow_rate_interval_lst), 2)          
                    average_interval_lst.append(average_flow_rate_interval)
                    _average = round(sum(average_interval_lst) / len(average_interval_lst), 1)
                readings = tuple([sampling_time, _average])
                
                # casting
                MQTT_config.pub_Topics[pub_Topic] = readings[-1]
                ## to slave data list
                slave.readings.append(readings)
                print(f'DFM_AOG_data_analyze done: record {readings} from slave_{slave.id}')

            except Exception as e7:
                MQTT_config.pub_Topics[analyze_err] += 1
                print('XX'*10 + f" {MQTT_config.pub_Topics[analyze_err]} DFM_AOG_data_analyze error at {round((time.time()-start),2)}s: " + str(e7) + 'XX'*5)
            finally:
                pass

    print('kill DFM_AOG_data_analyze')
    print(f'Final DFM_AOG_data_analyze: {MQTT_config.pub_Topics[analyze_err]} errors occured')
    #barrier_kill.wait()


def Scale_data_analyze(start, slave, pub_Topic, analyze_err):
    while (not config.kb_event.isSet()) and (not config.ticker.wait(config.sample_time_Scale)):
        lst_readings = slave.lst_readings
        time_readings = slave.time_readings
        #print(f'Scale_data_analyze: {lst_readings}')
        slave.lst_readings = []
        try:
            if len(lst_readings) > 0:
                #print(len(lst_readings))
                lst_readings = [sum(i)/len(i) for i in lst_readings] # average for 1s' data
                lst_readings = round((lst_readings[-1] - lst_readings[0]) / config.sample_time_Scale, 3) # average for 1min's data
                readings = tuple([round(time_readings, 2), lst_readings])
                
                # casting
                MQTT_config.pub_Topics[pub_Topic] = readings[-1]
                ## to slave data list
                slave.readings.append(readings)
                print(f'Scale_data_analyze done: record {readings} from slave_{slave.id}')
                #barrier_analyze.wait()
            else:
                print(f'Scale_data_analyze done: record () from slave_{slave.id}')
        except Exception as e:
            MQTT_config.pub_Topics[analyze_err] += 1
            print('XX'*10 + f" Scale_data_analyze error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[analyze_err]} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*5)
        finally:
            pass

    print(f'kill Scale_data_analyze of slave_{slave.id}')
    print(f'Final Scale_data_analyze: from slave_{slave.id}, {MQTT_config.pub_Topics[analyze_err]} errors occured')
    #barrier_kill.wait()


def GA_data_analyze(start, slave, pub_Topic, analyze_err):
    while not config.kb_event.isSet():
        if not config.ticker.wait(config.sample_time):
            lst_readings = slave.lst_readings
            time_readings = slave.time_readings
            #print(f'GA_data_analyze: {lst_readings}')
            slave.lst_readings = []
            slave.time_readings = []
            try:           
                arr_readings = np.array(
                    [[int(readings[i:i+4],16)/100 for i in range(8,20,4)] 
                    + [int(readings[24:28],16)/100] 
                    + [int(readings[-12:-8],16)/100] 
                    + [(lambda i: ((i[0]*256+i[1]+i[2])*256+i[3])/100)([int(readings[i:i+2],16) for i in range(-20,-12,2)])] 
                    for readings in lst_readings]
                    )
                #print(arr_readings)
                lst_readings = tuple(np.round(np.sum(arr_readings, axis=0) / len(lst_readings), 1))
                readings = tuple([round(time_readings[-1],2)]) + lst_readings
                
                # casting
                MQTT_config.pub_Topics[pub_Topic] = readings[-1]
                ## to slave data list
                slave.readings.append(readings)
                print(f'GA_data_analyze done: record {readings} from slave_{slave.id}')

            except Exception as e:
                MQTT_config.pub_Topics[analyze_err] += 1
                print('XX'*10 + f" {MQTT_config.pub_Topics[analyze_err]} GA_analyze error at {round((time.time()-start),2)}s: " + str(e) + 'XX'*5)
            finally:
                pass

    print('kill GA_data_analyze')
    print(f'Final GA_data_analyze: {MQTT_config.pub_Topics[analyze_err]} errors occured')
    #barrier_kill.wait()


def TCHeader_analyze(start, slave, pub_Topic, analyze_err):
    while (not config.kb_event.isSet()) and (not config.ticker.wait(config.sample_time)):
        #print(slave.id, slave.time_readings, slave.lst_readings)
        lst_readings = slave.lst_readings
        time_readings = slave.time_readings
        #print(slave.id, lst_readings)
        slave.lst_readings = []
        try:
            if len(lst_readings) > 0:
                arr_readings = np.array(
                    [int(readings[-8:-4],16)/10 # convert from hex to dec 
                    for readings in lst_readings]
                    )
                #print(slave.id, arr_readings)
                #print(slave.id, time_readings)
                lst_readings = tuple([np.round(np.sum(arr_readings) / len(lst_readings), 1)])
                readings = tuple([round(time_readings,2)]) + lst_readings
                #print(slave.id, readings)

                # casting
                MQTT_config.pub_Topics[pub_Topic] = readings[-1]
                ## to slave data list
                slave.readings.append(readings)
                print(f'TCHeader_analyze done: record {readings} from slave_{slave.id}')
                #barrier_analyze.wait()
            else:
                print(f'TCHeader_analyze done: record () from slave_{slave.id}')
        except Exception as e:
            MQTT_config.pub_Topics[analyze_err] += 1
            print('XX'*10 + f"TCHeader_analyze error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[analyze_err]} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*5)
        finally:
            pass

    print(f'kill TCHeader_analyze of slave_{slave.id}')
    print(f'Final TCHeader_analyze: from slave_{slave.id}, {MQTT_config.pub_Topics[analyze_err]} errors occured')
    #barrier_kill.wait()


def ADAM_SET_analyze(start, slave, pub_Topic, analyze_err):
    while (not config.kb_event.isSet()) and (not config.ticker.wait(config.sample_time)):
        #print(slave.id, slave.time_readings, slave.lst_readings)
        lst_readings = slave.lst_readings
        time_readings = slave.time_readings
        #print(slave.id, lst_readings)
        slave.lst_readings = []
        try:
            if len(lst_readings) > 0:
                print(lst_readings)
                arr_readings = np.array(
                    [int(readings[-8:-4],16)/(2**12)*20-10 # convert from hex to dec 
                    for readings in lst_readings]
                    )
                #print(slave.id, arr_readings)
                #print(slave.id, time_readings)
                lst_readings = tuple([np.round(np.sum(arr_readings) / len(lst_readings), 1)])
                readings = tuple([round(time_readings,2)]) + lst_readings
                print(lst_readings)
                print(slave.id, readings)

                # casting
                MQTT_config.pub_Topics[pub_Topic] = readings[-1]
                ## to slave data list
                slave.readings.append(readings)
                print(f'ADAM_analyze done: record {readings} from slave_{slave.id}')
                #barrier_analyze.wait()
            else:
                print(f'ADAM_analyze done: record () from slave_{slave.id}')
        except Exception as e:
            MQTT_config.pub_Topics[analyze_err] += 1
            print('XX'*10 + f"ADAM_analyze error: from slave_{slave.id}, err_{MQTT_config.pub_Topics[analyze_err]} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*5)
        finally:
            pass

    print(f'kill ADAM_analyze of slave_{slave.id}')
    print(f'Final ADAM_analyze: from slave_{slave.id}, {MQTT_config.pub_Topics[analyze_err]} errors occured')
    #barrier_kill.wait()
