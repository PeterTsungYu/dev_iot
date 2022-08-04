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
import PIDsim

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

def analyze_decker(func):
    def wrapper(start, device_port, slave):
        analyze_err = device_port.err_values[f'{slave.name}_analyze_err']
        _lst_readings = slave.lst_readings
        _time_readings = slave.time_readings
        slave.lst_readings = []
        if device_port.name == 'GPIO_port':
            cond = len(_time_readings)
            slave.time_readings = []
        else:    
            cond = len(_lst_readings)
        try:
            if cond > 0:
                analyze_err[1] += 1
                _readings = func(start, device_port, slave, _lst_readings=_lst_readings, _time_readings=_time_readings)
                # casting
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    device_port.pub_values[topic] = _readings[ind]
                    #logging.debug(device_port.pub_values)
                    ind += 1
                ## to slave data list
                slave.readings.append(_readings)
                logging.info(f"{slave.name}_analyze done: record {_readings}")
            elif (device_port.name == 'GPIO_port') and (cond == 0):
                analyze_err[1] += 1
                _readings = tuple([_time_readings, 0])
                # casting
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    device_port.pub_values[topic] = _readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(_readings)
                logging.info(f"{slave.name}_analyze done: record {_readings}")
            else:
                logging.warning(f"{slave.name}_analyze record nothing")
        except Exception as e:
            analyze_err[0] += 1
            logging.error(f"{slave.name}_analyze_err_{analyze_err} at {round((time.time()-start),2)}s: " + str(e))
        finally:
            logging.info(f"{slave.name}_analyze_err: {round((analyze_err[1] - analyze_err[0])/(analyze_err[1] + 0.00000000000000001)*100, 2)}%")
    return wrapper

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
            logging.info(f'Read {readings} from slave_{slave.name}')
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
        logging.info(f"{slave.name}_collect_err: {round((collect_err[1] - collect_err[0])/(collect_err[1]+0.00000000000000001)*100, 2)}%")


def Modbus_Comm(start, device_port, slave):    
    port = device_port.port
    collect_err = device_port.err_values.get(f'{slave.name}_collect_err')
    set_err = device_port.err_values.get(f'{slave.name}_set_err')
    re_collect = device_port.recur_count.get(f'{slave.name}_collect_err')
    re_set = device_port.recur_count.get(f'{slave.name}_set_err')
    try: # try to collect
        collect_err[1] += 1
        #logging.debug(slave.r_rtu)
        #logging.debug(bytes.fromhex(slave.r_rtu))
        port.write(bytes.fromhex(slave.r_rtu)) #hex to binary(byte) 
        slave.time_readings = time.time()-start
        time.sleep(params.time_out)
        #logging.debug(port.inWaiting())
        _data_len = port.inWaiting()
        if _data_len >= slave.r_wait_len: 
            readings = port.read(_data_len).hex() # after reading, the buffer will be clean        
            #logging.debug(readings)
            if slave.name == 'GA':
                slave.lst_readings.append(readings)
                logging.info(f'Read from slave_{slave.name}')
            else:    
                re = hex(int(slave.id))[2:].zfill(2) + '03' + hex(slave.r_wait_len-5)[2:].zfill(2)
                #logging.debug(readings.index(re))
                if readings.index(re) >= 0:
                    readings = readings[readings.index(re):(readings.index(re)+slave.r_wait_len*2)]
                logging.debug(readings)
                crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
                #logging.debug(crc)
                # check sta, func code, datalen, crc
                if (crc[-2:] + crc[:2]) == readings[-4:]:
                    slave.lst_readings.append(readings)
                    logging.info(f'Read from slave_{slave.name}')
                else:
                    collect_err[0] += 1
                    err_msg = f"{slave.name}_collect_err_{collect_err} at {round((time.time()-start),2)}s: crc validation failed"
                    logging.error(err_msg)
        elif _data_len == 0:
            # recursive part if the rtu was transferred but crushed in between the lines
            collect_err[2] += 1
            err_msg = f"{slave.name}_collect_err_{collect_err} at {round((time.time()-start),2)}s: wrong rtu code led to null receiving"
            logging.error(err_msg)
            if re_collect[0] < 3:
                re_collect[0] += 1
                logging.debug(re_collect)
                Modbus_Comm(start, device_port, slave)
        else: # if data len is less than the wait data
            collect_err[0] += 1
            err_msg = f"{slave.name}_collect_err_{collect_err} at {round((time.time()-start),2)}s: data len_{_data_len} is less than the wait data"
            logging.error(err_msg)
    except Exception as e:
        collect_err[0] += 1
        err_msg = f"{slave.name}_collect_err_{collect_err} at {round((time.time()-start),2)}s: " + str(e)
        logging.error(err_msg)
    finally:
        port.reset_input_buffer()
        logging.info(f"{slave.name}_collect_err: {round((collect_err[1] - collect_err[0])/(collect_err[1]+0.00000000000000001)*100, 2)}%")

    w_data_site=0
    for topic in slave.port_topics.sub_topics:
        #logging.critical((slave.name, topic))
        if device_port.sub_events[topic].isSet():
            #logging.critical(device_port.sub_values[topic])
            try: # try to set value
                set_err[1] += 1
                slave.write_rtu(f"{w_data_site:0>4}", device_port.sub_values[topic])
                port.write(bytes.fromhex(slave.w_rtu)) #hex to binary(byte) 
                time.sleep(params.time_out)
                _data_len = port.inWaiting()
                if _data_len >= slave.w_wait_len:
                    readings = port.read(_data_len).hex() # after reading, the buffer will be clean
                    #logging.critical(readings)
                    re = hex(int(slave.id))[2:].zfill(2) + '06' + f"{w_data_site:0>4}"
                    #logging.critical(re)
                    if readings.index(re):
                        readings = readings[readings.index(re):(readings.index(re)+slave.w_wait_len*2)]
                    #logging.critical(readings)
                    crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
                    # check sta, func code, datalen, crc
                    if (crc[-2:] + crc[:2]) == readings[-4:]:
                        logging.critical(readings)
                        logging.critical(f'Read from slave_{slave.name}')
                        device_port.sub_events[topic].clear()
                    else:
                        set_err[0] += 1
                        err_msg = f"{slave.name}_set_err_{set_err} at {round((time.time()-start),2)}s: crc validation failed"
                        logging.error(err_msg)
                else: # if data len is less than the wait data
                    set_err[0] += 1
                    err_msg = f"{slave.name}_set_err_{set_err} at {round((time.time()-start),2)}s: data len_{_data_len} is less than the wait data"
                    logging.error(err_msg)
            except Exception as e:
                set_err[0] += 1
                err_msg = f"{slave.name}_set_err_{set_err} at {round((time.time()-start),2)}s: " + str(e)
                logging.error(err_msg)
            finally:
                logging.info(f"{slave.name}_set_err: {round((set_err[1] - set_err[0])/(set_err[1]+0.00000000000000001)*100, 2)}%")
                port.reset_input_buffer()
                w_data_site += 1
        else:
            w_data_site += 1
        

def MFC_Comm(start, device_port, slave):    
    port = device_port.port
    collect_err = device_port.err_values.get(f'{slave.name}_collect_err')
    set_err = device_port.err_values.get(f'{slave.name}_set_err')
    re_collect = device_port.recur_count.get(f'{slave.name}_collect_err')
    try: # try to collect
        collect_err[1] += 1
        #logging.debug(slave.r_rtu)
        #logging.debug(bytes(slave.r_rtu, 'ASCII'))
        port.write(bytes(slave.r_rtu, 'ASCII')) #ASCII to byte
        slave.time_readings = time.time()-start
        time.sleep(params.time_out)
        #logging.debug(port.inWaiting())
        _data_len = port.inWaiting()
        if _data_len >= slave.r_wait_len: 
            readings = str(port.read(_data_len)) # after reading, the buffer will be clean
            #logging.debug(readings)
            # validate received data
            re = slave.id
            #logging.debug(readings.index(re))
            if readings.index(re) >= 0:
                readings = readings[readings.index(re):(readings.index(re)+slave.r_wait_len)]
                logging.debug(f'collect: {readings}')
                slave.lst_readings.append(readings)
                logging.info(f'Read from slave_{slave.name}')
            else:
                collect_err[0] += 1
                err_msg = f"{slave.name}_collect_err_{collect_err} at {round((time.time()-start),2)}s: validation failed"
                logging.error(err_msg)
        elif _data_len == 0:
            # recursive part if the rtu was transferred but crushed in between the lines
            collect_err[2] += 1
            err_msg = f"{slave.name}_collect_err_{collect_err} at {round((time.time()-start),2)}s: wrong rtu code led to null receiving"
            logging.error(err_msg)
            if re_collect[0] < 3:
                re_collect[0] += 1
                logging.debug(re_collect)
                MFC_Comm(start, device_port, slave)
        else: # if data len is less than the wait data
            collect_err[0] += 1
            err_msg = f"{slave.name}_collect_err_{collect_err} at {round((time.time()-start),2)}s: data len_{_data_len} is less than the wait data"
            logging.error(err_msg)
    except Exception as e:
        collect_err[0] += 1
        err_msg = f"{slave.name}_collect_err_{collect_err} at {round((time.time()-start),2)}s: " + str(e)
        logging.error(err_msg)
    finally:
        port.reset_input_buffer()
        logging.info(f"{slave.name}_collect_err: {round((collect_err[1] - collect_err[0])/(collect_err[1]+0.00000000000000001)*100, 2)}%")

    w_data_site=0
    for topic in slave.port_topics.sub_topics:
        #logging.critical((slave.name, topic))
        if device_port.sub_events[topic].isSet():
            #logging.debug(device_port.sub_values[topic])
            try: # try to set value
                set_err[1] += 1
                slave.write_rtu(device_port.sub_values[topic])
                #logging.debug(slave.w_rtu)
                port.write(bytes(slave.w_rtu, 'ASCII')) 
                time.sleep(params.time_out)
                _data_len = port.inWaiting()
                #logging.debug(_data_len)
                if _data_len >= slave.w_wait_len:
                    readings = str(port.read(_data_len)) # after reading, the buffer will be clean
                    re = slave.id
                    #logging.critical(re)
                    if readings.index(re)  >= 0:
                        readings = readings[readings.index(re):(readings.index(re)+slave.w_wait_len)]
                        logging.debug(f'write: {readings}')
                        logging.info(f'Read from slave_{slave.name}')
                        device_port.sub_events[topic].clear()
                    else:
                        set_err[0] += 1
                        err_msg = f"{slave.name}_set_err_{set_err} at {round((time.time()-start),2)}s: validation failed"
                        logging.error(err_msg)
                else: # if data len is less than the wait data
                    set_err[0] += 1
                    err_msg = f"{slave.name}_set_err_{set_err} at {round((time.time()-start),2)}s: data len_{_data_len} is less than the wait data"
                    logging.error(err_msg)
            except Exception as e:
                set_err[0] += 1
                err_msg = f"{slave.name}_set_err_{set_err} at {round((time.time()-start),2)}s: " + str(e)
                logging.error(err_msg)
            finally:
                logging.info(f"{slave.name}_set_err: {round((set_err[1] - set_err[0])/(set_err[1]+0.00000000000000001)*100, 2)}%")
                port.reset_input_buffer()
                w_data_site += 1
        else:
            w_data_site += 1


@kb_event
@sampling_event(params.sample_time)
@analyze_decker
def ADAM_TC_analyze(start, device_port, slave, _lst_readings, _time_readings):
    arr_readings = np.array([[int(reading[i-4:i],16) for i in range(10,len(reading)-2,4)] for reading in _lst_readings])
    _lst_readings = tuple(np.round(1370/65535*(np.sum(arr_readings, axis=0) / len(_lst_readings)), 1))
    _readings = tuple([round(_time_readings,2)]) + _lst_readings
    return _readings


@kb_event
@sampling_event(params.sample_time)
@analyze_decker
def ADAM_READ_analyze(start, device_port, slave, **kwargs):
    _lst_readings = kwargs.get('_lst_readings')
    logging.debug(_lst_readings)
    _time_readings = kwargs.get('_time_readings')
    _arr_readings = np.array([[int(reading[i-4:i],16) for i in range(10,len(reading)-2,4)] for reading in _lst_readings])
    logging.debug(_arr_readings)
    _lst_readings = np.sum(_arr_readings, axis=0) / len(_lst_readings)
    logging.debug(_lst_readings)
    _readings = tuple([round(_time_readings,2)]) + tuple(np.round(_lst_readings, 3))
    return _readings
    

@kb_event
@sampling_event(params.sample_time_DFM)
@analyze_decker
def DFM_data_analyze(start, device_port, slave, **kwargs):
    _time_readings = kwargs.get('_time_readings')
    _sampling_time = round(time.time()-start, 2)
    # _flow_rate_interval_lst = []
    # _average_interval_lst = []
    # calc average min flow rate by each interval
    try: 
        # flow rate in [liter/s]
        # 0.1 liter / pulse
        _flow_rate = 60 * 0.1 * (len(_time_readings) - 1) / (_time_readings[-1] - _time_readings[0])
        # _flow_rate_interval_lst.append(round(_flow_rate, 2)) 
        # _average_flow_rate_interval = round(sum(_flow_rate_interval_lst) / len(_flow_rate_interval_lst), 2)         
        # _average_interval_lst.append(_average_flow_rate_interval)
        # _average = round(sum(_average_interval_lst) / len(_average_interval_lst), 1)
        _readings = tuple([_sampling_time, round(_flow_rate, 2)])
    except:
        _readings = tuple([_sampling_time, 0])
    return _readings
            

@kb_event
@sampling_event(params.sample_time_DFM)
@analyze_decker
def DFM_AOG_data_analyze(start, device_port, slave, **kwargs):
    _time_readings = kwargs.get('_time_readings')
    _sampling_time = round(time.time()-start, 2)
    # _average_interval_lst = []
    # _flow_rate_interval_lst = []
    # calc average min flow rate by each interval 
    try:
        # flow rate in [liter/s]
        # 0.01 liter / pulse
        #print(_time_readings)
        _flow_rate = 60 * 0.01 * (len(_time_readings) - 1) / (_time_readings[-1] - _time_readings[0])
        #print(_flow_rate)
        # _flow_rate_interval_lst.append(round(_flow_rate, 2)) 
        # print(_flow_rate_interval_lst)
        # _average_flow_rate_interval = round(sum(_flow_rate_interval_lst) / len(_flow_rate_interval_lst), 2)          
        # print(_average_flow_rate_interval)
        # _average_interval_lst.append(_average_flow_rate_interval)
        # print(_average_interval_lst)
        # _average = round(sum(_average_interval_lst) / len(_average_interval_lst), 1)
        _readings = tuple([_sampling_time, round(_flow_rate,2)])
        print(_readings)
    except:
        _readings = tuple([_sampling_time, 0])
    return _readings
            

@kb_event
@sampling_event(params.sample_time)
@analyze_decker
def Scale_data_analyze(start, device_port, slave, **kwargs):
    _lst_readings = kwargs.get('_lst_readings')
    _time_readings = kwargs.get('_time_readings')
    _arr_readings = np.array([sum(i)/len(i) for i in _lst_readings])
    _lst_readings = tuple([np.sum(_arr_readings) / len(_lst_readings)])
    _readings = tuple([round(_time_readings,2)]) + _lst_readings
    return _readings

@kb_event
@sampling_event(params.sample_time)
@analyze_decker
def GA_data_analyze(start, device_port, slave, **kwargs):
    _lst_readings = kwargs.get('_lst_readings')
    _time_readings = kwargs.get('_time_readings')
    print(_lst_readings)
    _arr_readings = np.array(
        [[int(readings[i:i+4],16)/100 if (int(readings[i:i+4],16)/100) <= 99.99 else 0 for i in range(8,20,4)] # CO, CO2, CH4
        + [int(readings[24:28],16)/100] # H2
        + [int(readings[-12:-8],16)/100] # N2
        + [(lambda i: ((i[0]*256+i[1]+i[2])*256+i[3])/100)([int(readings[i:i+2],16) for i in range(-20,-12,2)])] # Heat
        for readings in _lst_readings]
        )
    print(_arr_readings)
    _lst_readings = tuple(np.round(np.sum(_arr_readings, axis=0) / len(_lst_readings), 1))
    print(_lst_readings)
    _readings = tuple([round(_time_readings,2)]) + _lst_readings
    return _readings
            

@kb_event
@sampling_event(params.sample_time)
@analyze_decker
def TCHeader_analyze(start, device_port, slave, **kwargs):
    _lst_readings = kwargs.get('_lst_readings')
    _time_readings = kwargs.get('_time_readings')
    _arr_readings = np.array(
        [int(readings[-8:-4],16) # convert from hex to dec 
        for readings in _lst_readings]
        )
    _lst_readings = tuple([np.sum(_arr_readings) / len(_lst_readings)])
    _readings = tuple([round(_time_readings,2)]) + _lst_readings
    return _readings


@kb_event
@sampling_event(params.sample_time)
@analyze_decker
def ADAM_SET_analyze(start, device_port, slave, **kwargs):
    _lst_readings = kwargs.get('_lst_readings')
    _time_readings = kwargs.get('_time_readings')
    _arr_readings = np.array(
        [[int(readings[6:-4][i:i+4],16)/(2**12)*20-10 for i in range(0,16,4)] # convert from hex to dec 
        for readings in _lst_readings]
        )
    _lst_readings = tuple(np.sum(_arr_readings, 0) / len(_lst_readings))
    _readings = tuple([round(_time_readings,2)]) + _lst_readings
    return _readings

@kb_event
@sampling_event(params.sample_time)
@analyze_decker
def Air_MFC_analyze(start, device_port, slave, **kwargs):
    _lst_readings = kwargs.get('_lst_readings')
    _time_readings = kwargs.get('_time_readings')
    _arr_readings = np.array([[float(i) for i in re.findall('\d+.\d+',readings)] for readings in _lst_readings])
    logging.debug(_arr_readings)
    _lst_readings = tuple(np.sum(_arr_readings, 0) / len(_lst_readings))
    _readings = tuple([round(_time_readings,2)]) + _lst_readings
    return _readings

@kb_event
@sampling_event(params.sample_time)
@analyze_decker
def H2_MFC_analyze(start, device_port, slave, **kwargs):
    _lst_readings = kwargs.get('_lst_readings')
    _time_readings = kwargs.get('_time_readings')
    _arr_readings = np.array([[float(i) for i in re.findall('\d+.\d+',readings)] for readings in _lst_readings])
    logging.debug(_arr_readings)
    _lst_readings = tuple(np.sum(_arr_readings, 0) / len(_lst_readings))
    _readings = tuple([round(_time_readings,2)]) + _lst_readings
    return _readings

def VOID(start, device_port, slave):
    pass

#------------------------------PID Control---------------------------------
'''
# reactor temperature setpoint
Tsp = 390

# set initial conditions and cooling flow
IC = [C0,T0,Tcf]

# do simulation at fixed time steps dt
tstart = 0
tstop = 8
tstep = 0.05

# configure controller. Creates a PID object.
reactorPID = PID(Kp=8,Ki=30,Kd=5,MVrange=(0,300),DirectAction=True)

c,T,Tc = IC                                          # reactor initial conditions
qc = 150                                             # initial condition of the MV
for t in np.arange(tstart,tstop,tstep):              # simulate from tstart to tstop
    qc = reactorPID.update(t,Tsp,T,qc)               # update manipulated variable
    c,T,Tc = odeint(deriv,[c,T,Tc],[t,t+dt])[-1]     # start at t, find state at t + dt
'''