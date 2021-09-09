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


def sampling_event(sample_time):
    def decker(func):
        def wrapper(*arg):
            if not params.ticker.wait(sample_time):
                func(*arg)
        return wrapper
    return decker

#------------------------------Collect and Analyze func---------------------------------
def Scale_data_collect(start, device_port, slave):
    #while not params.kb_event.isSet():
    port = device_port.port
    collect_err = device_port.err_values[f'{slave.name}_collect_err']
    try:
        collect_err[1] += 1
        slave.time_readings = time.time()-start
        time.sleep(params.time_out) # wait for the data input to the buffer
        if port.inWaiting() > slave.r_wait_len:
            readings = port.read(port.inWaiting()).decode('utf-8')
            readings = [float(s) if s[0] != '-' else -float(s[1:]) for s in re.findall(r'[ \-][ .\d]{7}', readings)]
            slave.lst_readings.append(readings)
            logging.info(f'Read from slave_{slave.name}')
            port.reset_input_buffer() # reset the buffer after each reading process
        else: # if data len is no data
            collect_err[0] += 1
            err_msg = f"{slave.name}_collect_err_{collect_err} at {round((time.time()-start),2)}s: data len is no wait data"
            logging.error(err_msg)
    except Exception as e:
        collect_err[0] += 1
        err_msg = f"{slave.name}_collect_err_{collect_err} at {round((time.time()-start),2)}s: " + str(e)
        logging.error(err_msg)
    finally:
        logging.info(f"{slave.name}_collect_err: {round((collect_err[1] - collect_err[0])/collect_err[1]*100, 2)}%")


def Modbus_Comm(start, device_port, slave):    
    port = device_port.port
    collect_err = device_port.err_values.get(f'{slave.name}_collect_err')
    set_err = device_port.err_values.get(f'{slave.name}_set_err')
    try: # try to collect
        collect_err[1] += 1
        port.write(bytes.fromhex(slave.r_rtu)) #hex to binary(byte) 
        slave.time_readings = time.time()-start
        time.sleep(params.time_out)
        logging.debug(port.inWaiting())    
        if port.inWaiting() >= slave.r_wait_len: 
            readings = port.read(port.inWaiting()).hex() # after reading, the buffer will be clean
            logging.debug(readings)
            re = hex(int(slave.id))[2:].zfill(2) + '03' + hex(slave.r_wait_len-5)[2:].zfill(2)
            logging.debug(re)
            if readings.index(re):
                readings = readings[readings.index(re):(readings.index(re)+slave.r_wait_len*2)]
            logging.debug(readings)
            crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
            # check sta, func code, datalen, crc
            if (crc[-2:] + crc[:2]) == readings[-4:]:
                slave.lst_readings.append(readings)
                logging.info(f'Read from slave_{slave.name}')
            else:
                port.reset_input_buffer() # reset the buffer if crc failed
                collect_err[0] += 1
                err_msg = f"{slave.name}_collect_err_{collect_err} at {round((time.time()-start),2)}s: crc validation failed"
                logging.error(err_msg)
        else: # if data len is less than the wait data
            port.reset_input_buffer() # reset the buffer if len is incorrect
            collect_err[0] += 1
            err_msg = f"{slave.name}_collect_err_{collect_err} at {round((time.time()-start),2)}s: data len is less than the wait data"
            logging.error(err_msg)
    except Exception as e:
        collect_err[0] += 1
        err_msg = f"{slave.name}_collect_err_{collect_err} at {round((time.time()-start),2)}s: " + str(e)
        logging.error(err_msg)
    finally:
        logging.info(f"{slave.name}_collect_err: {round((collect_err[1] - collect_err[0])/collect_err[1]*100, 2)}%")

    for topic in slave.port_topics.sub_topics:
        w_data_site=0
        if device_port.sub_events[topic].isSet():
            try: # try to set value
                set_err[1] += 1
                slave.write_rtu(f"{w_data_site:0>4}", device_port.sub_values[topic])
                port.write(bytes.fromhex(slave.w_rtu)) #hex to binary(byte) 
                time.sleep(params.time_out)
                if port.inWaiting() >= slave.w_wait_len:
                    readings = port.read(port.inWaiting()).hex() # after reading, the buffer will be clean
                    logging.debug(readings)
                    re = hex(int(slave.id))[2:].zfill(2) + '06' + hex(slave.w_wait_len-5)[2:].zfill(2)
                    logging.debug(re)
                    if readings.index(re):
                        readings = readings[readings.index(re):(readings.index(re)+slave.w_wait_len*2)]
                    logging.debug(readings)
                    crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
                    # check sta, func code, datalen, crc
                    if (crc[-2:] + crc[:2]) == readings[-4:]:
                        slave.lst_readings.append(readings)
                        logging.info(f'Read from slave_{slave.name}')
                    else:
                        port.reset_input_buffer() # reset the buffer if no read
                        set_err[0] += 1
                        err_msg = f"{slave.name}_set_err_{set_err} at {round((time.time()-start),2)}s: crc validation failed"
                        logging.error(err_msg)
                else: # if data len is less than the wait data
                    port.reset_input_buffer() # reset the buffer if no read
                    set_err[0] += 1
                    err_msg = f"{slave.name}_set_err_{set_err} at {round((time.time()-start),2)}s: data len is less than the wait data"
                    logging.error(err_msg)
            except Exception as e:
                set_err[0] += 1
                err_msg = f"{slave.name}_set_err_{set_err} at {round((time.time()-start),2)}s: " + str(e)
                logging.error(err_msg)
            finally:
                device_port.sub_events[topic].clear()
                logging.info(f"{slave.name}_set_err: {round((set_err[1] - set_err[0])/set_err[1]*100, 2)}%")
        else:
            pass
        w_data_site += 1


@kb_event
@sampling_event(params.sample_time)
def ADAM_TC_analyze(start, device_port, slave):
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
@sampling_event(params.sample_time)
def ADAM_READ_analyze(start, device_port, slave):
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
@sampling_event(params.sample_time_DFM)
def DFM_data_analyze(start, device_port, slave):
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
@sampling_event(params.sample_time_DFM)
def DFM_AOG_data_analyze(start, device_port, slave):
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
@sampling_event(params.sample_time_Scale)
def Scale_data_analyze(start, device_port, slave):
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
@sampling_event(params.sample_time)
def GA_data_analyze(start, device_port, slave):
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
@sampling_event(params.sample_time)
def TCHeader_analyze(start, device_port, slave):
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
@sampling_event(params.sample_time)
def ADAM_SET_analyze(start, device_port, slave):
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