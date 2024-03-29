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
logger.setLevel(logging.ERROR)
formatter = logging.Formatter(
	'[%(levelname)s %(asctime)s %(module)s:%(lineno)d] %(message)s',
	datefmt='%Y%m%d %H:%M:%S')

ch = logging.StreamHandler()
ch.setLevel(logging.ERROR)
ch.setFormatter(formatter)

fh = logging.FileHandler(filename='platform.log', mode='w')
fh.setLevel(logging.ERROR)
fh.setFormatter(formatter)

logger.addHandler(ch)
logger.addHandler(fh)

#------------------------------Decker---------------------------------
def kb_event(func):
    def wrapper(*arg):
        while not params.kb_event.is_set():
            func(*arg)
    return wrapper


def sampling_event():
    def decker(func):
        def wrapper(*arg):
            params.sample_ticker.wait()
            func(*arg)
        return wrapper
    return decker


def analyze_decker(func):
    def wrapper(start, device_port, slave):
        analyze_err = device_port.err_values[f'{slave.name}_analyze_err']
        slave.lst_readings.put(None)
        slave.time_readings.put(None)
        _lst_readings = list(iter(slave.lst_readings.get, None))
        _time_readings = list(iter(slave.time_readings.get, None))
        if slave.name in ['Scale']:
            _size_lst = slave.size_lst_readings
            _size_time = slave.size_time_readings
            _size_lst['short_lst_readings'].append(_lst_readings)
            _size_lst['long_lst_readings'].append(_lst_readings)
            _size_time['short_time_readings'].append(_time_readings)
            _size_time['long_time_readings'].append(_time_readings)
            if len(_size_lst['short_lst_readings']) > 10: # aggregate lists for 10s in a list
                _size_lst['short_lst_readings'] = params.manager.list(_size_lst['short_lst_readings'][-10:])
                _size_time['short_time_readings'] = params.manager.list(_size_time['short_time_readings'][-10:])
            if len(_size_lst['long_lst_readings']) > 60: # aggregate lists for 60s in a list
                _size_lst['long_lst_readings'] = params.manager.list(_size_lst['long_lst_readings'][-60:])
                _size_time['long_time_readings'] = params.manager.list(_size_time['long_time_readings'][-60:])
            _lst_readings = _size_lst
            _time_readings = _size_time
            cond = len(_lst_readings['short_lst_readings'])
        elif 'ADAM_TC' in slave.name:
            _size_lst = slave.size_lst_readings
            _size_time = slave.size_time_readings
            _size_lst['short_lst_readings'].append(_lst_readings)
            _size_time['short_time_readings'].append(_time_readings)
            if len(_size_lst['short_lst_readings']) > 5: # aggregate lists for 10s in a list
                _size_lst['short_lst_readings'] = params.manager.list(_size_lst['short_lst_readings'][-5:])
                _size_time['short_time_readings'] = params.manager.list(_size_time['short_time_readings'][-5:])
            _lst_readings = _size_lst
            _time_readings = _size_time
            cond = len(_lst_readings['short_lst_readings'])
        elif 'DFM' in slave.name:
            #print(_time_readings)
            _DFM_time = slave.size_time_readings
            _DFM_time['short_time_readings'].append(_time_readings)
            _DFM_time['long_time_readings'].append(_time_readings)
            if len(_DFM_time['short_time_readings']) > 10: # aggregate lists for 10s in a list
                _DFM_time['short_time_readings'] = params.manager.list(_DFM_time['short_time_readings'][-10:])
            if len(_DFM_time['long_time_readings']) > 60: # aggregate lists for 60s in a list
                _DFM_time['long_time_readings'] = params.manager.list(_DFM_time['long_time_readings'][-60:])
            _lst_readings = []
            _time_readings = _DFM_time
            cond = len(_DFM_time['short_time_readings'])
            # print(cond)
        else:    
            cond = len(_lst_readings)
        try:
            analyze_err[1] += 1
            if cond > 0:
                _readings = func(start, device_port, slave, _lst_readings=_lst_readings, _time_readings=_time_readings)
                # casting
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    device_port.pub_values[topic].value = _readings[ind]
                    #logging.critical(f'{topic} {device_port.pub_values[topic].value}')
                    ind += 1
                ## to slave data list
                #slave.readings.append(_readings)
                analyze_err[2] += 1
                logging.debug(f"{slave.name}_analyze done: record {_readings}")
            elif (device_port.name == 'GPIO_port') and (cond == 0):
                _readings = tuple([_time_readings, 0])
                # casting
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    device_port.pub_values[topic].value = _readings[ind]
                    #logging.debug(f'{topic} {device_port.pub_values[topic].value}')
                    ind += 1
                ## to slave data list
                #slave.readings.append(_readings)
                analyze_err[2] += 1
                logging.debug(f"{slave.name}_analyze done: record {_readings}")
            else:
                analyze_err[0] += 1
                logging.warning(f"{slave.name}_analyze record nothing")
        except Exception as e:
            analyze_err[0] += 1
            logging.debug(f"{slave.name}_analyze_err_{analyze_err[:]} at {round((time.time()-start),2)}s: " + str(e))
        finally:
            logging.debug(f"{slave.name}_analyze_err: {round(analyze_err[2]/(analyze_err[1] + 0.00000000000000001)*100, 2)}%, analyze_err_{analyze_err[:]}")
    return wrapper

#------------------------------Collect and Analyze func---------------------------------
def Scale_data_collect(start, device_port, slave):
    port = device_port.port
    collect_err = device_port.err_values[f'{slave.name}_collect_err']
    analyze_switch = False
    try:
        collect_err[1] += 1
        time.sleep(slave.timeout) # wait for the data input to the buffer
        if port.inWaiting() > slave.r_wait_len:
            readings = port.read(port.inWaiting()).decode('utf-8')
            readings = [float(s) if s[0] != '-' else -float(s[1:]) for s in re.findall(r'[ \-][ .\d]{7}', readings)]
            slave.time_readings.put(time.time()-start)
            slave.lst_readings.put(readings)
            collect_err[2] += 1
            logging.debug(f'Read {readings} from slave_{slave.name}')
            port.reset_input_buffer() # reset the buffer after each reading process
            analyze_switch = True
        else: # if data len is no data
            collect_err[0] += 1
            err_msg = f"{slave.name}_collect_err_{collect_err[:]} at {round((time.time()-start),2)}s: data len is no wait data"
            logging.error(err_msg)
    except Exception as e:
        collect_err[0] += 1
        err_msg = f"{slave.name}_collect_err_{collect_err[:]} at {round((time.time()-start),2)}s: " + str(e)
        logging.error(err_msg)
    finally:
        logging.debug(f"{slave.name}_collect_err: {round(collect_err[2]/(collect_err[1]+0.00000000000000001)*100, 2)}%, collect_err_{collect_err[:]}")
        # if (collect_err[1] >= params.exempt_try) and (round(collect_err[2]/(collect_err[1]+0.00000000000000001)*100, 2) <= params.exempt_threshold):
        #     if slave.name not in device_port.broken_slave_names:
        #         device_port.broken_slave_names.append(slave.name)
        if analyze_switch:
            if slave.kwargs.get('analyze_func'):
                slave.kwargs['analyze_func'](start, device_port, slave)


def Modbus_Comm(start, device_port, slave):    
    port = device_port.port
    collect_err = device_port.err_values.get(f'{slave.name}_collect_err')
    set_err = device_port.err_values.get(f'{slave.name}_set_err')
    re_collect = device_port.recur_count.get(f'{slave.name}_collect_err')
    re_set = device_port.recur_count.get(f'{slave.name}_set_err')
    analyze_switch = False
    try: # try to collect
        recur = False
        collect_err[1] += 1
        #logging.error(slave.r_rtu)
        # logging.debug(bytes.fromhex(slave.r_rtu))
        port.write(bytes.fromhex(slave.r_rtu)) #hex to binary(byte) 
        time.sleep(slave.timeout)
        #logging.debug(port.inWaiting())
        _data_len = port.inWaiting()
        if _data_len >= slave.r_wait_len: 
            readings = port.read(_data_len).hex() # after reading, the buffer will be clean        
            #logging.debug(readings)
            if slave.name == 'GA':
                slave.time_readings.put(time.time()-start)
                slave.lst_readings.put(readings)
                logging.debug(f'Read from slave_{slave.name}')
                analyze_switch = True
            else:    
                re = hex(int(slave.id))[2:].zfill(2) + '03' + hex(slave.r_wait_len-5)[2:].zfill(2)
                #logging.debug(readings.index(re))
                if re in readings:
                    readings = readings[readings.index(re):(readings.index(re)+slave.r_wait_len*2)]
                    logging.debug(readings)
                    crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
                    #logging.debug(crc)
                    # check sta, func code, datalen, crc
                    if (crc[-2:] + crc[:2]) == readings[-4:]:
                        slave.time_readings.put(time.time()-start)
                        slave.lst_readings.put(readings)
                        collect_err[2] += 1
                        logging.debug(f'Read from slave_{slave.name}')
                        analyze_switch = True
                    else:
                        recur = True
                        collect_err[0] += 1
                        err_msg = f"{slave.name}_collect_err_{collect_err[:]} at {round((time.time()-start),2)}s: crc validation failed"
                        logging.error(err_msg)
                else:
                    recur = True
                    collect_err[0] += 1
                    err_msg = f"{slave.name}_collect_err_{collect_err[:]} at {round((time.time()-start),2)}s: Modbus protocol failed"
                    logging.error(err_msg)
        else: # if data len is less than the wait data
            if _data_len == 0:
                recur = True
                collect_err[0] += 1
                err_msg = f"{slave.name}_collect_err_{collect_err[:]} at {round((time.time()-start),2)}s: wrong rtu code led to null receiving"
                logging.error(err_msg)
            elif _data_len < slave.r_wait_len:
                recur = True
                collect_err[0] += 1
                err_msg = f"{port.read(_data_len).hex()} {slave.name}_collect_err_{collect_err[:]} at {round((time.time()-start),2)}s: data len_{_data_len} is less than the wait data"
                logging.error(err_msg)
    except Exception as e:
        collect_err[0] += 1
        err_msg = f"{slave.name}_collect_err_{collect_err[:]} at {round((time.time()-start),2)}s: " + str(e)
        logging.error(err_msg)
    finally:
        # recursive part if the rtu was transferred but crushed in between the lines
        if (recur == True) and (re_collect[0] < params.recur_try):
            re_collect[0] += 1
            re_collect[1] += 1
            logging.debug(f're_collect: {re_collect[:]}')
            logging.debug(f'collect_err: {collect_err[:]}')
            Modbus_Comm(start, device_port, slave)
        re_collect[0] = 0
        port.reset_input_buffer()
        logging.debug(f"{slave.name}_collect_err: {round(collect_err[2]/(collect_err[1]+0.00000000000000001)*100, 2)}%, recollect_cover: {round(re_collect[1]/(collect_err[0]+0.00000000000000001)*100, 2)}%")
        # if (collect_err[1] >= params.exempt_try) and (round(collect_err[2]/(collect_err[1]+0.00000000000000001)*100, 2) <= params.exempt_threshold):
        #     if slave.name not in device_port.broken_slave_names:
        #         device_port.broken_slave_names.append(slave.name)
        if analyze_switch:
            if slave.kwargs.get('analyze_func'):
                slave.kwargs['analyze_func'](start, device_port, slave)

    w_data_site=0
    for topic in slave.port_topics.sub_topics:
        #logging.critical((slave.name, topic))
        if device_port.sub_events[topic].is_set():
            #logging.critical(device_port.sub_values[topic])
            try: # try to set value
                recur = False
                set_err[1] += 1
                slave.write_rtu(f"{w_data_site:0>4}", device_port.sub_values[topic].value)
                port.write(bytes.fromhex(slave.w_rtu)) #hex to binary(byte) 
                time.sleep(slave.timeout)
                _data_len = port.inWaiting()
                if _data_len >= slave.w_wait_len:
                    readings = port.read(_data_len).hex() # after reading, the buffer will be clean
                    #logging.critical(readings)
                    re = hex(int(slave.id))[2:].zfill(2) + '06' + f"{w_data_site:0>4}"
                    #logging.critical(re)
                    if re in readings:
                        readings = readings[readings.index(re):(readings.index(re)+slave.w_wait_len*2)]
                        #logging.critical(readings)
                        crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
                        # check sta, func code, datalen, crc
                        if (crc[-2:] + crc[:2]) == readings[-4:]:
                            set_err[2] += 1
                            device_port.sub_events[topic].clear()
                            logging.debug(f'write: {readings}')
                        else:
                            recur = True
                            set_err[0] += 1
                            err_msg = f"{slave.name}_set_err_{set_err[:]} at {round((time.time()-start),2)}s: crc validation failed"
                            logging.error(err_msg)
                    else:
                        recur = True
                        set_err[0] += 1
                        err_msg = f"{slave.name}_set_err_{set_err[:]} at {round((time.time()-start),2)}s: Modbus protocol failed"
                        logging.error(err_msg)
                else: # if data len is less than the wait data
                    if _data_len == 0:
                        recur = True
                        set_err[0] += 1
                        err_msg = f"{slave.name}_set_err_{set_err[:]} at {round((time.time()-start),2)}s: wrong rtu code led to null receiving"
                        logging.error(err_msg)
                    elif _data_len < slave.w_wait_len:
                        recur = True
                        set_err[0] += 1
                        err_msg = f"{port.read(_data_len).hex()} {slave.name}_set_err_{set_err[:]} at {round((time.time()-start),2)}s: data len_{_data_len} is less than the wait data"
                        logging.error(err_msg)
            except Exception as e:
                set_err[0] += 1
                err_msg = f"{slave.name}_set_err_{set_err[:]} at {round((time.time()-start),2)}s: " + str(e)
                logging.error(err_msg)
            finally:
                # recursive part if the rtu was transferred but crushed in between the lines
                if (recur == True) and (re_set[0] < params.recur_try):
                    re_set[0] += 1
                    re_set[1] += 1
                    logging.debug(f're_set: {re_set[:]}')
                    Modbus_Comm(start, device_port, slave)
                re_set[0] = 0
                port.reset_input_buffer()
                logging.debug(f"{slave.name}_set_err: {round(set_err[2]/(set_err[1]+0.00000000000000001)*100, 2)}%, reset_cover: {round(re_set[1]/(collect_err[0]+0.00000000000000001)*100, 2)}%")
                w_data_site += 1
                # if (set_err[1] >= params.exempt_try) and (round(set_err[2]/(set_err[1]+0.00000000000000001)*100, 2) <= params.exempt_threshold):
                #     if slave.name not in device_port.broken_slave_names:
                #         device_port.broken_slave_names.append(slave.name)
        else:
            w_data_site += 1
        

def MFC_Comm(start, device_port, slave):    
    port = device_port.port
    collect_err = device_port.err_values.get(f'{slave.name}_collect_err')
    set_err = device_port.err_values.get(f'{slave.name}_set_err')
    re_collect = device_port.recur_count.get(f'{slave.name}_collect_err')
    re_set = device_port.recur_count.get(f'{slave.name}_set_err')
    analyze_switch = False
    try: # try to collect
        recur = False
        collect_err[1] += 1
        #logging.debug(slave.r_rtu)
        #logging.debug(bytes(slave.r_rtu, 'ASCII'))
        port.write(bytes(slave.r_rtu, 'ASCII')) #ASCII to byte
        time.sleep(slave.timeout)
        #logging.debug(port.inWaiting())
        _data_len = port.inWaiting()
        if _data_len >= slave.r_wait_len: 
            readings = str(port.read(_data_len)) # after reading, the buffer will be clean
            logging.debug(readings)
            # validate received data
            re_id = slave.id
            #logging.debug(readings.index(re))
            if re_id in readings:
                readings = readings[readings.index(re_id):(readings.index(re_id)+slave.r_wait_len)]
                if re.findall('\d+.\d+',readings):
                    logging.debug(f'collect: {readings}')
                    slave.time_readings.put(time.time()-start)
                    slave.lst_readings.put(readings)
                    collect_err[2] += 1
                    logging.debug(f'Read from slave_{slave.name}')
                    analyze_switch = True
                else:
                    recur = True
                    collect_err[0] += 1
                    err_msg = f"{slave.name}_collect_err_{collect_err[:]} at {round((time.time()-start),2)}s: validation failed (slave_id conflicted with gas name in resp)"
                    logging.error(err_msg)        
            else:
                recur = True
                collect_err[0] += 1
                err_msg = f"{slave.name}_collect_err_{collect_err[:]} at {round((time.time()-start),2)}s: validation failed"
                logging.error(err_msg)    
        else: # if data len is less than the wait data
            if _data_len == 0:
                recur = True
                collect_err[0] += 1
                err_msg = f"{slave.name}_collect_err_{collect_err[:]} at {round((time.time()-start),2)}s: wrong rtu code led to null receiving"
                logging.error(err_msg)
            elif _data_len < slave.r_wait_len:
                recur = True
                collect_err[0] += 1
                err_msg = f"{slave.name}_collect_err_{collect_err[:]} at {round((time.time()-start),2)}s: data len_{_data_len} is less than the wait data"
                logging.error(err_msg)
    except Exception as e:
        collect_err[0] += 1
        err_msg = f"{slave.name}_collect_err_{collect_err[:]} at {round((time.time()-start),2)}s: " + str(e)
        logging.error(err_msg)
    finally:
        # recursive part if the rtu was transferred but crushed in between the lines
        if (recur == True) and (re_collect[0] < params.recur_try):
            re_collect[0] += 1
            re_collect[1] += 1
            logging.debug(f're_collect: {re_collect[:]}')
            MFC_Comm(start, device_port, slave)
        re_collect[0] = 0
        port.reset_input_buffer()
        logging.debug(f"{slave.name}_collect_err: {round(collect_err[2]/(collect_err[1]+0.00000000000000001)*100, 2)}%, recollect_cover: {round(re_collect[1]/(collect_err[0]+0.00000000000000001)*100, 2)}%")
        # if (collect_err[1] >= params.exempt_try) and (round(collect_err[2]/(collect_err[1]+0.00000000000000001)*100, 2) <= params.exempt_threshold):
        #     if slave.name not in device_port.broken_slave_names:
        #         device_port.broken_slave_names.append(slave.name)
        if analyze_switch:
            if slave.kwargs.get('analyze_func'):
                slave.kwargs['analyze_func'](start, device_port, slave)

    w_data_site=0
    for topic in slave.port_topics.sub_topics:
        #logging.critical((slave.name, topic))
        if device_port.sub_events[topic].is_set():
            #logging.debug(device_port.sub_values[topic])
            try: # try to set value
                recur = False
                set_err[1] += 1
                slave.write_rtu(device_port.sub_values[topic].value)
                #logging.debug(slave.w_rtu)
                port.write(bytes(slave.w_rtu, 'ASCII')) 
                time.sleep(slave.timeout)
                _data_len = port.inWaiting()
                #logging.debug(_data_len)
                if _data_len >= slave.w_wait_len:
                    readings = str(port.read(_data_len)) # after reading, the buffer will be clean
                    re_id = slave.id
                    #logging.critical(re)
                    if re_id in readings:
                        readings = readings[readings.index(re_id):(readings.index(re_id)+slave.w_wait_len)]
                        if re.findall('\d+.\d+',readings):
                            set_err[2] += 1
                            device_port.sub_events[topic].clear()
                            logging.debug(f'write: {readings}')
                        else:
                            recur = True
                            set_err[0] += 1
                            err_msg = f"{slave.name}_set_err_{set_err[:]} at {round((time.time()-start),2)}s: validation failed (slave_id conflicted with gas name in resp)"
                            logging.error(err_msg)    
                    else:
                        recur = True
                        set_err[0] += 1
                        err_msg = f"{slave.name}_set_err_{set_err[:]} at {round((time.time()-start),2)}s: validation failed"
                        logging.error(err_msg)
                else: # if data len is less than the wait data
                    if _data_len == 0:
                        recur = True
                        set_err[0] += 1
                        err_msg = f"{slave.name}_set_err_{set_err[:]} at {round((time.time()-start),2)}s: wrong rtu code led to null receiving"
                        logging.error(err_msg)
                    elif _data_len < slave.w_wait_len:
                        recur = True
                        set_err[0] += 1
                        err_msg = f"{port.read(_data_len).hex()} {slave.name}_set_err_{set_err[:]} at {round((time.time()-start),2)}s: data len_{_data_len} is less than the wait data"
                        logging.error(err_msg)
            except Exception as e:
                set_err[0] += 1
                err_msg = f"{slave.name}_set_err_{set_err[:]} at {round((time.time()-start),2)}s: " + str(e)
                logging.error(err_msg)
            finally:
                # recursive part if the rtu was transferred but crushed in between the lines
                if (recur == True) and (re_set[0] < params.recur_try):
                    re_set[0] += 1
                    logging.debug(f're_set: {re_set[:]}')
                    MFC_Comm(start, device_port, slave)
                re_set[0] = 0
                port.reset_input_buffer()
                logging.debug(f"{slave.name}_set_err: {round(set_err[2]/(set_err[1]+0.00000000000000001)*100, 2)}%, reset_cover: {round(re_set[1]/(collect_err[0]+0.00000000000000001)*100, 2)}%")
                w_data_site += 1
                # if (set_err[1] >= params.exempt_try) and (round(set_err[2]/(set_err[1]+0.00000000000000001)*100, 2) <= params.exempt_threshold):
                #     if slave.name not in device_port.broken_slave_names:
                #         device_port.broken_slave_names.append(slave.name)
        else:
            w_data_site += 1


@analyze_decker
def ADAM_TC_analyze(start, device_port, slave, **kwargs):
    _lst_readings = kwargs.get('_lst_readings')['short_lst_readings']
    _time_readings = kwargs.get('_time_readings')['short_time_readings']
    #print(_time_readings)
    _lst = []
    _time = []
    for i in range(0, len(_lst_readings)):
        if _lst_readings[i]:
            _time.extend(_time_readings[i])
            _lst.extend(_lst_readings[i])
    if _lst:
        arr_readings = np.array([[int(reading[i-4:i],16) for i in range(10,len(reading)-2,4)] for reading in _lst])
        arr_readings = tuple(np.round(1370/65535*arr_readings, 2))
        _last = tuple([round(_time[-1],2)]) + tuple(arr_readings[-1])
        
        if _time[0] == _time[-1]:
            _5_rates = tuple([0]*8)
        else:
            _5_rates = tuple(np.round((arr_readings[-1] - arr_readings[0]) / (_time[-1] - _time[0]),2)) # BR_rate, SR_rate
    else:
        _last = tuple([round(_time[-1],2)]) + tuple([0]*8)
        _5_rates = tuple([0]*8)
    _readings = _last + _5_rates[0:2]  
    return _readings


@analyze_decker
def ADAM_READ_analyze(start, device_port, slave, **kwargs):
    _lst_readings = kwargs.get('_lst_readings')
    logging.debug(_lst_readings)
    _time_readings = kwargs.get('_time_readings')[-1]
    _arr_readings = np.array([[int(reading[i-4:i],16) for i in range(10,len(reading)-2,4)] for reading in _lst_readings])
    logging.debug(_arr_readings)
    _lst_readings = np.sum(_arr_readings, axis=0) / len(_lst_readings)
    logging.debug(_lst_readings)
    _readings = tuple([round(_time_readings,2)]) + tuple(np.round(_lst_readings, 3))
    return _readings
    

@analyze_decker
def DFM_data_analyze(start, device_port, slave, **kwargs):
    _time_readings = kwargs.get('_time_readings')
    _sampling_time = round(time.time()-start, 2)
    _10_flow_lst = []
    try:
        for i in range(len(_time_readings['short_time_readings'])):
            if i != [] and (len(_time_readings['short_time_readings'][i]) > 1):
                _10_flow_lst.append((len(_time_readings['short_time_readings'][i]) - 1) / (_time_readings['short_time_readings'][i][-1] - _time_readings['short_time_readings'][i][0]))
            else:
                _10_flow_lst.append(0)
        _10_flow_rate = sum(_10_flow_lst) / len(_10_flow_lst)
        if slave.name == 'DFM':
            _10_flow_rate = _10_flow_rate * 10 * 0.1
        elif slave.name == 'DFM_AOG':
            _10_flow_rate = _10_flow_rate * 10 * 0.01

        _60_flow_lst = []
        for i in range(len(_time_readings['long_time_readings'])):
            if i != [] and (len(_time_readings['long_time_readings'][i]) > 1):
                _60_flow_lst.append((len(_time_readings['long_time_readings'][i]) - 1) / (_time_readings['long_time_readings'][i][-1] - _time_readings['long_time_readings'][i][0]))
            else:
                _60_flow_lst.append(0)
        _60_flow_rate = sum(_60_flow_lst) / len(_60_flow_lst)
        if slave.name == 'DFM':
            _60_flow_rate = _60_flow_rate * 60 * 0.1
        elif slave.name == 'DFM_AOG':
            _60_flow_rate = _60_flow_rate * 60 * 0.01
        _readings = tuple([_sampling_time, round(_10_flow_rate,2), round(_60_flow_rate,2)])
    except:
        _readings = tuple([_sampling_time, 0, 0])
        #print(_readings)
    return _readings


@analyze_decker
def Scale_data_analyze(start, device_port, slave, **kwargs):
    _sampling_time = round(time.time()-start, 2)
    _lst_readings = kwargs.get('_lst_readings')
    _time_readings = kwargs.get('_time_readings')
    _10_scale_lst = []
    _10_scale_time = []
    # print(_lst_readings['short_lst_readings'])
    # print(_time_readings['short_time_readings'])
    assert len(_lst_readings['short_lst_readings']) == len(_time_readings['short_time_readings'])
    for i in range(0, len(_lst_readings['short_lst_readings'])):
        _10_scale_lst.extend(_lst_readings['short_lst_readings'][i])
        _10_scale_time.extend(_time_readings['short_time_readings'][i])
    for i in _10_scale_lst: 
        if -0.00001 < i < 0.00001:
            _10_scale_lst.remove(i)
            _10_scale_time.remove(i)
    if _10_scale_lst:
        _10_scale = (_10_scale_lst[0] - _10_scale_lst[-1]) / (_10_scale_time[-1] - _10_scale_time[0]) * 1000 * 10
    else:
        _10_scale = 0
    # print(_10_scale)
    
    _60_scale_lst = []
    _60_scale_time = []
    assert len(_lst_readings['long_lst_readings']) == len(_time_readings['long_time_readings'])
    for i in range(0, len(_lst_readings['long_lst_readings'])):
        _60_scale_lst.extend(_lst_readings['long_lst_readings'][i])
        _60_scale_time.extend(_time_readings['long_time_readings'][i])
    for i in _60_scale_lst: 
        if -0.00001 < i < 0.00001:
            _60_scale_lst.remove(i)
            _60_scale_time.remove(i)
    if _60_scale_lst:
        _60_scale = (_60_scale_lst[0] - _60_scale_lst[-1]) / (_60_scale_time[-1] - _60_scale_time[0]) * 1000 * 60
    else:
        _60_scale = 0
        
    _readings = tuple([round(_sampling_time,2), round(_10_scale, 2), round(_60_scale, 2)])
    return _readings


@analyze_decker
def GA_data_analyze(start, device_port, slave, **kwargs):
    _lst_readings = kwargs.get('_lst_readings')
    _time_readings = kwargs.get('_time_readings')
    #print(_lst_readings)
    if _lst_readings and _time_readings:
        _arr_readings = np.array(
            [[int(readings[i:i+4],16)/100 if (int(readings[i:i+4],16)/100) <= 99.99 else 0 for i in range(8,20,4)] # CO, CO2, CH4
            + [int(readings[24:28],16)/100] # H2
            + [int(readings[-12:-8],16)/100] # N2
            + [(lambda i: ((i[0]*256+i[1]+i[2])*256+i[3])/100)([int(readings[i:i+2],16) for i in range(-20,-12,2)])] # Heat
            for readings in _lst_readings]
            )
        #print(_arr_readings)
        _lst_readings = tuple(np.round(np.sum(_arr_readings, axis=0) / len(_lst_readings), 1))
    else:
        _lst_readings = tuple([0]*6)
    _readings = tuple([round(_time_readings[-1],2)]) + _lst_readings
    return _readings
            

@analyze_decker
def TCHeader_analyze(start, device_port, slave, **kwargs):
    _lst_readings = kwargs.get('_lst_readings')
    _time_readings = kwargs.get('_time_readings')[-1]
    _arr_readings = np.array(
        [int(readings[-8:-4],16) # convert from hex to dec 
        for readings in _lst_readings]
        )
    _lst_readings = tuple([np.sum(_arr_readings) / len(_lst_readings)])
    _readings = tuple([round(_time_readings,2)]) + _lst_readings
    return _readings


@analyze_decker
def ADAM_SET_analyze(start, device_port, slave, **kwargs):
    _lst_readings = kwargs.get('_lst_readings')
    _time_readings = kwargs.get('_time_readings')[-1]
    _arr_readings = np.array(
        [[int(readings[6:-4][i:i+4],16)/(2**12)*20-10 for i in range(0,16,4)] # convert from hex to dec 
        for readings in _lst_readings]
        )
    _lst_readings = tuple(np.sum(_arr_readings, 0) / len(_lst_readings))
    _readings = tuple([round(_time_readings,2)]) + _lst_readings
    return _readings


@analyze_decker
def MFC_analyze(start, device_port, slave, **kwargs):
    _lst_readings = kwargs.get('_lst_readings')
    _time_readings = kwargs.get('_time_readings')[-1]
    _arr_readings = np.array([[float(i) for i in re.findall('\d+.\d+',readings)] for readings in _lst_readings], dtype=object)
    #print(_arr_readings)
    _lst_readings = tuple(np.sum(_arr_readings, 0) / len(_lst_readings))
    _readings = tuple([round(_time_readings,2)]) + _lst_readings
    return _readings

def VOID(start, device_port, slave):
    time.sleep(slave.timeout)

#------------------------------PID controller---------------------------------
@kb_event
def control(device_port, slave):
    _update_parameter = False
    for topic in slave.port_topics.sub_topics:
        if topic in [f'{slave.name}_Kp', f'{slave.name}_Ki', f'{slave.name}_Kd', f'{slave.name}_MVmin',  f'{slave.name}_MVmax', f'{slave.name}_mode', f'{slave.name}_beta', f'{slave.name}_tstep', f'{slave.name}_kick']:
            if device_port.sub_events[topic].is_set():
                _update_parameter = True
                device_port.sub_events[topic].clear()
            
    _sub_values = device_port.sub_values
    Kp = _sub_values.get(f'{slave.name}_Kp').value
    Ki = _sub_values.get(f'{slave.name}_Ki').value
    Kd = _sub_values.get(f'{slave.name}_Kd').value
    MVmin = _sub_values.get(f'{slave.name}_MVmin').value
    MVmax = _sub_values.get(f'{slave.name}_MVmax').value
    mode = _sub_values.get(f'{slave.name}_mode').value
    SP = _sub_values.get(f'{slave.name}_SP').value
    PV = _sub_values.get(f'{slave.name}_PV').value
    MV = _sub_values.get(f'{slave.name}_setting').value
    beta = _sub_values.get(f'{slave.name}_beta').value
    kick = _sub_values.get(f'{slave.name}_kick').value
    tstep = _sub_values.get(f'{slave.name}_tstep').value
    if kick is None:
        kick = 1
    if tstep is None or tstep == 0:
        tstep = 1
    if _update_parameter:
        slave.controller.update_paramater(Kp=Kp, Ki=Ki, Kd=Kd, MVmin=MVmin, MVmax=MVmax, mode=mode, beta=beta)
    try:
        #print(slave.name, tstep, SP, PV, MV, kick)
        updates = slave.controller.update(tstep, SP, PV, MV, kick)
        for idx, topic in enumerate(slave.port_topics.pub_topics):    
            device_port.pub_values[topic].value = updates[idx]
    except Exception as e:
        logging.error(f'{e}')
    # print(tstep)
    time.sleep(tstep)