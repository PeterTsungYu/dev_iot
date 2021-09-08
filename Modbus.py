# pip3 install crccheck
# pip3 install numpy 
## for numpy, also do this: sudo apt-get install libatlas-base-dev
# pip3 install pyserial

#python packages
import numpy as np
import time
import re
from crccheck.crc import Crc16Modbus
import logging

#custom modules
import params

#------------------------------Logger---------------------------------
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
formatter = logging.Formatter(
	'[%(levelname)s %(asctime)s %(module)s:%(lineno)d] %(message)s',
	datefmt='%Y%m%d %H:%M:%S')

ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
ch.setFormatter(formatter)

fh = logging.FileHandler(filename='platform.log', mode='w')
fh.setLevel(logging.DEBUG)
fh.setFormatter(formatter)

logger.addHandler(ch)
logger.addHandler(fh)

#------------------------------Decker---------------------------------
def kb_event(func):
    def wrapper(*arg):
        while not params.kb_event.isSet():
            func(*arg)
    return wrapper

#------------------------------Collect and Analyze func---------------------------------
def Modbus_Read(start, device_port, slave):
    #while not params.kb_event.isSet():
    port = device_port.port
    try:
        port.write(bytes.fromhex(slave.r_rtu)) #hex to binary(byte) 
        
        slave.time_readings = time.time()-start

        time.sleep(params.time_out)

        # look up the buffer for 21 bytes, which is for 8 channels data length
        if port.inWaiting() >= slave.r_wait_len: 
            readings = port.read(slave.r_wait_len).hex() # after reading, the buffer will be clean
            crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
            # check sta, func code, datalen, crc
            if (readings[0:2] == slave.id) and (readings[2:4] == '03') and ((crc[-2:] + crc[:2]) == readings[-4:]):
                slave.lst_readings.append(readings)
                #print(readings)
                logging.info(f'Read from slave_{slave.name}')
            else:
                port.reset_input_buffer() # reset the buffer if no read
                device_port.err_values[f'{slave.name}_collect_err'] += 1
                err_msg = f"{slave.name}_collect_err_{device_port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: crc validation failed"
                logging.error(err_msg)
        else: # if data len is less than the wait data
            port.reset_input_buffer() # reset the buffer if no read
            device_port.err_values[f'{slave.name}_collect_err'] += 1
            err_msg = f"{slave.name}_collect_err_{device_port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: data len is less than the wait data"
            logging.error(err_msg)
    except Exception as e:
        device_port.err_values[f'{slave.name}_collect_err'] += 1
        err_msg = f"{slave.name}_collect_err_{device_port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: " + str(e)
        logging.error(err_msg)
    finally:
        pass


def Scale_data_collect(start, device_port, slave):
    #while not params.kb_event.isSet():
    port = device_port.port
    try:
        slave.time_readings = time.time()-start
        time.sleep(params.time_out) # wait for the data input to the buffer
        if port.inWaiting() > slave.r_wait_len:
            readings = port.read(port.inWaiting()).decode('utf-8')
            readings = [float(s) if s[0] != '-' else -float(s[1:]) for s in re.findall(r'[ \-][ .\d]{7}', readings)]
            slave.lst_readings.append(readings)
            logging.info(f'Read from slave_{slave.name}')
            port.reset_input_buffer() # reset the buffer after each reading process
        else: # if data len is no data
            device_port.err_values[f'{slave.name}_collect_err'] += 1
            err_msg = f"{slave.name}_collect_err_{device_port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: data len is no wait data"
            logging.error(err_msg)
    except Exception as e:
        device_port.err_values[f'{slave.name}_collect_err'] += 1
        err_msg = f"{slave.name}_collect_err_{device_port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: " + str(e)
        logging.error(err_msg)
    finally:
        pass


def Modbus_Comm(start, device_port, slave):    
    #while not params.kb_event.isSet(): # it is written in the ReadingThreads.py
    port = device_port.port
    for topic in slave.port_topics.sub_topics:
        w_data_site=0
        if not device_port.sub_events[topic].isSet():
            try: # try to collect
                port.write(bytes.fromhex(slave.r_rtu)) #hex to binary(byte) 
                
                slave.time_readings = time.time()-start

                time.sleep(params.time_out)

                #print(port.inWaiting())    
                if port.inWaiting() >= slave.r_wait_len: 
                    readings = port.read(slave.r_wait_len).hex() # after reading, the buffer will be clean
                    crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
                    # check sta, func code, datalen, crc
                    if (readings[0:2] == slave.id) and (readings[2:4] == '03') and ((crc[-2:] + crc[:2]) == readings[-4:]):
                        slave.lst_readings.append(readings)
                        logging.info(f'Read from slave_{slave.name}')
                    else:
                        port.reset_input_buffer() # reset the buffer if crc failed
                        device_port.err_values[f'{slave.name}_collect_err'] += 1
                        err_msg = f"{slave.name}_collect_err_{device_port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: crc validation failed"
                        logging.error(err_msg)
                else: # if data len is less than the wait data
                    port.reset_input_buffer() # reset the buffer if len is incorrect
                    device_port.err_values[f'{slave.name}_collect_err'] += 1
                    err_msg = f"{slave.name}_collect_err_{device_port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: data len is less than the wait data"
                    logging.error(err_msg)
            except Exception as e:
                device_port.err_values[f'{slave.name}_collect_err'] += 1
                err_msg = f"{slave.name}_collect_err_{device_port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: " + str(e)
                logging.error(err_msg)
            finally:
                pass

        else:
            try: # try to set value
                slave.write_rtu(f"{w_data_site:0>4}", device_port.sub_values[topic])
                port.write(bytes.fromhex(slave.w_rtu)) #hex to binary(byte) 
                
                time.sleep(params.time_out)

                if port.inWaiting() >= slave.w_wait_len:
                    readings = port.read(slave.w_wait_len).hex() # after reading, the buffer will be clean
                    #print(readings)
                    crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
                    # check sta, func code, datalen, crc
                    if (readings[0:2] == slave.id) and (readings[2:4] == '06') and ((crc[-2:] + crc[:2]) == readings[-4:]):
                        logging.info(f'Write to slave_{slave.name}')
                    else:
                        port.reset_input_buffer() # reset the buffer if no read
                        device_port.err_values[f'{slave.name}_collect_err'] += 1
                        err_msg = f"{slave.name}_collect_err_{device_port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: crc validation failed"
                        logging.error(err_msg)
                else: # if data len is less than the wait data
                    port.reset_input_buffer() # reset the buffer if no read
                    device_port.err_values[f'{slave.name}_collect_err'] += 1
                    err_msg = f"{slave.name}_collect_err_{device_port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: data len is less than the wait data"
                    logging.error(err_msg)
            except Exception as e:
                device_port.err_values[f'{slave.name}_collect_err'] += 1
                err_msg = f"{slave.name}_collect_err_{device_port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: " + str(e)
                logging.error(err_msg)
            finally:
                device_port.sub_events[topic].clear()
        
        w_data_site += 1


@kb_event
def ADAM_TC_analyze(start, device_port, slave):
    #while not params.kb_event.isSet():
    if not params.ticker.wait(params.sample_time):
        lst_readings = slave.lst_readings
        time_readings = slave.time_readings
        #print(f'ADAM_TC_analyze: {lst_readings}')
        slave.lst_readings = []
        try:
            if len(lst_readings) > 0:
                arr_readings = np.array([[int(reading[i-4:i],16) for i in range(10,len(reading)-2,4)] for reading in lst_readings])
                lst_readings = tuple(np.round(1370/65535*(np.sum(arr_readings, axis=0) / len(lst_readings)), 1))
                #print(lst_readings)
                readings = tuple([round(time_readings,2)]) + lst_readings
                #print(readings)

                # casting
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    device_port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                logging.info(f"{slave.name}_analyze done: record {readings}")
            else:
                logging.warning(f"{slave.name}_analyze record nothing")

        except Exception as e:
            device_port.err_values[f'{slave.name}_analyze_err'] += 1
            logging.error(f"{slave.name}_analyze_err_{device_port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e))
        finally:
            pass


@kb_event
def ADAM_READ_analyze(start, device_port, slave):
    #while not params.kb_event.isSet():
    if not params.ticker.wait(params.sample_time):
        lst_readings = slave.lst_readings
        time_readings = slave.time_readings
        #print(f'ADAM_Read_analyze: {lst_readings}')
        slave.lst_readings = []
        try:
            if len(lst_readings) > 0:
                arr_readings = np.array([[int(reading[i-4:i],16) for i in range(10,len(reading)-2,4)] for reading in lst_readings])
                #print(arr_readings)
                lst_readings = np.sum(arr_readings, axis=0) / len(lst_readings)
                lst_readings[0] = lst_readings[0] / 65536 * 5
                lst_readings[1] = lst_readings[1] / 65536
                lst_readings[4] = (lst_readings[4] - 32767) / 32767 * 120
                lst_readings[5] = (lst_readings[5] - 32767) / 32767 * 250
                lst_readings[6] = (lst_readings[6] - 32767) / 32767 * 100
                #print(lst_readings)
                readings = tuple([round(time_readings,2)]) + tuple(np.round(lst_readings, 3))
                #print(readings)

                # casting
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    device_port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                logging.info(f"{slave.name}_analyze done: record {readings}")
            else:
                logging.warning(f"{slave.name}_analyze record nothing")

        except Exception as e:
            device_port.err_values[f'{slave.name}_analyze_err'] += 1
            logging.error(f"{slave.name}_analyze_err_{device_port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e))
        finally:
            pass


@kb_event
def DFM_data_analyze(start, device_port, slave):
    #while not params.kb_event.isSet():
    if not params.ticker.wait(params.sample_time_DFM): # for each sample_time, collect data
        sampling_time = round(time.time()-start, 2)
        try: 
            time_readings = slave.time_readings

            if len(time_readings) > 0:
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
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    device_port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                logging.info(f"{slave.name}_analyze done: record {readings}")
            else:
                logging.warning(f"{slave.name}_analyze record nothing")

        except Exception as e:
            device_port.err_values[f'{slave.name}_analyze_err'] += 1
            logging.error(f"{slave.name}_analyze_err_{device_port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e))
        finally:
            pass


@kb_event
def DFM_AOG_data_analyze(start, device_port, slave):
    #while not params.kb_event.isSet():
    if not params.ticker.wait(params.sample_time_DFM): # for each sample_time, collect data
        sampling_time = round(time.time()-start, 2)
        try: 
            time_readings = slave.time_readings

            if len(time_readings) > 0:
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
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    device_port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                logging.info(f"{slave.name}_analyze done: record {readings}")
            else:
                logging.warning(f"{slave.name}_analyze record nothing")

        except Exception as e:
            device_port.err_values[f'{slave.name}_analyze_err'] += 1
            logging.error(f"{slave.name}_analyze_err_{device_port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e))
        finally:
            pass


@kb_event
def Scale_data_analyze(start, device_port, slave):
    #while (not params.kb_event.isSet()): 
    if not params.ticker.wait(params.sample_time_Scale):
        lst_readings = slave.lst_readings
        time_readings = slave.time_readings
        #print(f'Scale_data_analyze: {lst_readings}')
        slave.lst_readings = []
        try:
            if len(lst_readings) > 0:
                #print(len(lst_readings))
                lst_readings = [sum(i)/len(i) for i in lst_readings] # average for 1s' data
                lst_readings = round((lst_readings[-1] - lst_readings[0]) / params.sample_time_Scale, 3) # average for 1min's data
                readings = tuple([round(time_readings, 2), lst_readings])
                
                # casting
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    device_port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                logging.info(f"{slave.name}_analyze done: record {readings}")
            else:
                logging.warning(f"{slave.name}_analyze record nothing")

        except Exception as e:
            device_port.err_values[f'{slave.name}_analyze_err'] += 1
            logging.error(f"{slave.name}_analyze_err_{device_port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e))
        finally:
            pass


@kb_event
def GA_data_analyze(start, device_port, slave):
    #while not params.kb_event.isSet():
    if not params.ticker.wait(params.sample_time):
        lst_readings = slave.lst_readings
        time_readings = slave.time_readings
        #print(f'GA_data_analyze: {lst_readings}')
        slave.lst_readings = []
        try:
            if len(lst_readings) > 0:           
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
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    device_port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                logging.info(f"{slave.name}_analyze done: record {readings}")
            else:
                logging.warning(f"{slave.name}_analyze record nothing")

        except Exception as e:
            device_port.err_values[f'{slave.name}_analyze_err'] += 1
            logging.error(f"{slave.name}_analyze_err_{device_port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e))
        finally:
            pass


@kb_event
def TCHeader_analyze(start, device_port, slave):
    #while (not params.kb_event.isSet()): 
    if not params.ticker.wait(params.sample_time):
        #print(slave.id, slave.time_readings, slave.lst_readings)
        lst_readings = slave.lst_readings
        time_readings = slave.time_readings
        #print(slave.id, lst_readings)
        slave.lst_readings = []
        #print(lst_readings)
        #print(time_readings)
        try:
            if len(lst_readings) > 0:
                arr_readings = np.array(
                    [int(readings[-8:-4],16) # convert from hex to dec 
                    for readings in lst_readings]
                    )
                #print(slave.id, arr_readings)
                #print(slave.id, time_readings)
                lst_readings = tuple([np.sum(arr_readings) / len(lst_readings)])
                readings = tuple([round(time_readings,2)]) + lst_readings
                #print(slave.id, readings)

                # casting
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    device_port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                logging.info(f"{slave.name}_analyze done: record {readings}")
            else:
                logging.warning(f"{slave.name}_analyze record nothing")

        except Exception as e:
            device_port.err_values[f'{slave.name}_analyze_err'] += 1
            logging.error(f"{slave.name}_analyze_err_{device_port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e))
        finally:
            pass


@kb_event
def ADAM_SET_analyze(start, device_port, slave):
    #while (not params.kb_event.isSet())
    if not params.ticker.wait(params.sample_time):
        #print(slave.id, slave.time_readings, slave.lst_readings)
        lst_readings = slave.lst_readings
        time_readings = slave.time_readings
        #print(slave.id, lst_readings)
        slave.lst_readings = []
        try:
            if len(lst_readings) > 0:
                #print(lst_readings)
                arr_readings = np.array(
                    [[int(readings[6:-4][i:i+4],16)/(2**12)*20-10 for i in range(0,16,4)] # convert from hex to dec 
                    for readings in lst_readings]
                    )
                #print(slave.id, arr_readings)
                #print(slave.id, time_readings)
                lst_readings = tuple(np.sum(arr_readings, 0) / len(lst_readings))
                #print(lst_readings)
                readings = tuple([round(time_readings,2)]) + lst_readings
                #print(lst_readings)
                #print(slave.id, readings)

                # casting
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    device_port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                logging.info(f"{slave.name}_analyze done: record {readings}")
            else:
                logging.warning(f"{slave.name}_analyze record nothing")

        except Exception as e:
            device_port.err_values[f'{slave.name}_analyze_err'] += 1
            logging.error(f"{slave.name}_analyze_err_{device_port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e))
        finally:
            pass
