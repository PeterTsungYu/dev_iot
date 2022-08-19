# pip3 install crccheck
# pip3 install numpy 
## for numpy, also do this: sudo apt-get install libatlas-base-dev
# pip3 install pyserial
# pip3 install -U minimalmodbus

#python packages
import numpy as np
import time
import re
from crccheck.crc import Crc16Modbus
import logging
import ADDA_ADS1256
import ADDA_DAC8532
import RPi.GPIO as GPIO
import pigpio

#custom modules
import params
import minimalmodbus
import serial

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

#-----ADDA port setting----------------------------------------------------------------
ADC = ADDA_ADS1256.ADS1256()
DAC = ADDA_DAC8532.DAC8532()
ADC.ADS1256_init()
DAC.DAC8532_Out_Voltage(ADDA_DAC8532.channel_A, 0)
DAC.DAC8532_Out_Voltage(ADDA_DAC8532.channel_B, 0)

channel_Relay01_IN1     = 24
channel_Relay01_IN2     = 25
GPIO.setmode(GPIO.BCM) # for software PWM
PIG = pigpio.pi() # for hardware PWM

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

def miniModbus_comm(start, device_port, slave):
    # print(device_port.name, slave.name)
    port = device_port.port
    collect_err = device_port.err_values.get(f'{slave.name}_collect_err')
    set_err = device_port.err_values.get(f'{slave.name}_set_err')
    re_collect = device_port.recur_count.get(f'{slave.name}_collect_err')
    re_set = device_port.recur_count.get(f'{slave.name}_set_err')

    # instrument adress
    # 4126 open/close 1/0
    # 4015 flow rate
    # 4022 unit 2 for ml/min
    # 4023 clockwise/anticlockwise 1/0
    # instrument.write_register(4026, 0, 0, 6, False)
    
    slave.time_readings = time.time()-start
    instrument = device_port.instrument
    XXX = minimalmodbus.Instrument(port.port, int(slave.id))
    XXX.read_register(4126, 3)
    if instrument.read_register(4126, 3) == 0:
        # append 0 to readings when instrument closed
        readings = 0
        slave.lst_readings.append(round(readings, 4))
        print('instrument close')
        time.sleep(params.time_out)
    else:
        # append ml/min to readings
        readings = instrument.read_float(4015 , 3)
        slave.lst_readings.append(round(readings, 4))
        time.sleep(params.time_out)

        pass
    for topic in slave.port_topics.sub_topics:
        logging.critical((slave.name, topic))
        logging.critical(device_port.sub_events[topic].isSet())
        if device_port.sub_events[topic].isSet():
            logging.critical(device_port.sub_values[topic])
            try: # try to set value
                collect_err[1] += 1
                slave.time_readings = time.time()-start
                time.sleep(params.time_out)
                if device_port.sub_values[topic] == 0:
                    instrument.write_register(4126, 0, 0, 6, False)
                    logging.info(f'Read {0} from slave_{slave.name}')
                    time.sleep(params.time_out)
                else:
                    instrument.write_register(4022, 2, 0, 6, False)
                    instrument.write_register(4023, 0, 0, 6, False) 
                    # instrument.read_register(4015)   
                    instrument.write_float(4015 ,float(device_port.sub_values[topic]) , 2, 0)
                    instrument.write_register(4126, 1, 0, 6, False)
                    logging.info(f'Read {readings} from slave_{slave.name}')
                    time.sleep(params.time_out)
                port.reset_input_buffer()
                device_port.sub_events[topic].clear()
            except Exception as e:
                set_err[0] += 1
                err_msg = f"{slave.name}_set_err_{set_err} at {round((time.time()-start),2)}s: " + str(e)
                logging.error(err_msg)
            finally:
                logging.info(f"{slave.name}_set_err: {round((set_err[1] - set_err[0])/(set_err[1]+0.00000000000000001)*100, 2)}%")
        time.sleep(params.time_out)
        # print(device_port.pub_values)
        # slave.lst_readings.append(round(readings, 4))


def PWM_comm(start, device_port, slave):
    port = device_port.port
    collect_err = device_port.err_values.get(f'{slave.name}_collect_err')
    set_err = device_port.err_values.get(f'{slave.name}_set_err')
    re_collect = device_port.recur_count.get(f'{slave.name}_collect_err')
    re_set = device_port.recur_count.get(f'{slave.name}_set_err')
    # set PWM to ON / OFF, frequency, duty 
    for topic in slave.port_topics.sub_topics:
        #logging.critical((slave.name, topic))
        if device_port.sub_events[topic].isSet():
            #logging.critical(device_port.sub_values[topic])
            try: # try to set value
                open_SV = device_port.sub_values[f'{slave.name}_open_SV']
                duty = device_port.sub_values[f'{slave.name}_duty_SV']
                f = device_port.sub_values[f'{slave.name}_f_SV']
                if open_SV:
                    #print(dir(slave.GPIO_instance))
                    #GPIO.setup(slave.id, GPIO.OUT, initial=0)
                    if slave.GPIO_instance:
                        slave.GPIO_instance.ChangeFrequency(f)
                        slave.GPIO_instance.ChangeDutyCycle(duty)
                    else:
                        slave.GPIO_instance = GPIO.PWM(slave.id, f)
                        slave.GPIO_instance.start(duty)
                    print(f"open at duty:{duty}, f: {f}")
                else:
                    if slave.GPIO_instance:
                        slave.GPIO_instance.stop()
                        # slave.GPIO_instance = GPIO.PWM(slave.id, 1)
                        # slave.GPIO_instance.start(0)
                    print(f"close at duty:{0}, f: {1}")
                for i in device_port.sub_events.values():
                    i.clear()
                print('clear flag')
            except Exception as e:
                set_err[0] += 1
                err_msg = f"{slave.name}_set_err_{set_err} at {round((time.time()-start),2)}s: " + str(e)
                logging.error(err_msg)
            finally:
                logging.info(f"{slave.name}_set_err: {round((set_err[1] - set_err[0])/(set_err[1]+0.00000000000000001)*100, 2)}%")


def PIG_PWM_comm(start, device_port, slave):
    port = device_port.port
    collect_err = device_port.err_values.get(f'{slave.name}_collect_err')
    set_err = device_port.err_values.get(f'{slave.name}_set_err')
    re_collect = device_port.recur_count.get(f'{slave.name}_collect_err')
    re_set = device_port.recur_count.get(f'{slave.name}_set_err')
    # set PWM to ON / OFF, frequency, duty 
    for topic in slave.port_topics.sub_topics:
        #logging.critical((slave.name, topic))
        if device_port.sub_events[topic].isSet():
            #logging.critical(device_port.sub_values[topic])
            try: # try to set value
                # print(topic)
                open_SV = device_port.sub_values[f'{slave.name}_open_SV']
                duty = device_port.sub_values[f'{slave.name}_duty_SV']
                f = device_port.sub_values[f'{slave.name}_f_SV']
                if open_SV:
                    PIG.hardware_PWM(slave.id, int(f * 1e3), int(duty * 1e4))
                    print(f"open at duty:{duty}%, f: {f}kHz")
                else:
                    PIG.write(slave.id, 0)
                    print('close')
                device_port.sub_events[topic].clear()
                print('clear flag')
            except Exception as e:
                PIG.write(slave.id, 0)
                set_err[0] += 1
                err_msg = f"{slave.name}_set_err_{set_err} at {round((time.time()-start),2)}s: " + str(e)
                logging.error(err_msg)
            finally:
                logging.info(f"{slave.name}_set_err: {round((set_err[1] - set_err[0])/(set_err[1]+0.00000000000000001)*100, 2)}%")


def Relay_comm(start, device_port, slave):
    port = device_port.port
    collect_err = device_port.err_values.get(f'{slave.name}_collect_err')
    set_err = device_port.err_values.get(f'{slave.name}_set_err')
    re_collect = device_port.recur_count.get(f'{slave.name}_collect_err')
    re_set = device_port.recur_count.get(f'{slave.name}_set_err')

    # set Relay to ON / OFF 
    for topic in slave.port_topics.sub_topics:
        #logging.critical((slave.name, topic))
        if device_port.sub_events[topic].isSet():
            #logging.critical(device_port.sub_values[topic])
            try: # try to set value
                if device_port.sub_values[topic]:
                    #GPIO.output(channel_Relay01_IN1, 0)
                    GPIO.output(channel_Relay01_IN2, 0)
                else:
                    #GPIO.output(channel_Relay01_IN1, 1)
                    GPIO.output(channel_Relay01_IN2, 1)
                device_port.sub_events[topic].clear()
            except Exception as e:
                set_err[0] += 1
                err_msg = f"{slave.name}_set_err_{set_err} at {round((time.time()-start),2)}s: " + str(e)
                logging.error(err_msg)
            finally:
                logging.info(f"{slave.name}_set_err: {round((set_err[1] - set_err[0])/(set_err[1]+0.00000000000000001)*100, 2)}%")


def ADDA_comm(start, device_port, slave):
    port = device_port.port
    collect_err = device_port.err_values.get(f'{slave.name}_collect_err')
    set_err = device_port.err_values.get(f'{slave.name}_set_err')
    re_collect = device_port.recur_count.get(f'{slave.name}_collect_err')
    re_set = device_port.recur_count.get(f'{slave.name}_set_err')
    
    #collect:
    #8ch 24bit high-precision ADC (4ch differential input), 30ksps sampling rate
    ADC_Value = ADC.ADS1256_GetAll()
    # print(ADC_Value)
    # print ("0 ADC = %lf"%(ADC_Value[0]*5.0/0x7fffff)) #32-bit integer in hexadecimal with all but the highest bit set
    # print ("1 ADC = %lf"%(ADC_Value[1]*5.0/0x7fffff))
    # print ("2 ADC = %lf"%(ADC_Value[2]*3.3/0x7fffff))
    # print ("3 ADC = %lf"%(ADC_Value[3]*3.3/0x7fffff))
    # print ("4 ADC = %lf"%(ADC_Value[4]*3.3/0x7fffff))
    # print ("5 ADC = %lf"%(ADC_Value[5]*3.3/0x7fffff))
    # print ("6 ADC = %lf"%(ADC_Value[6]*3.3/0x7fffff))
    # print ("7 ADC = %lf"%(ADC_Value[7]*3.3/0x7fffff))

    for topic in slave.port_topics.sub_topics:
        #logging.critical((slave.name, topic))
        if device_port.sub_events[topic].isSet():
            #logging.critical(device_port.sub_values[topic])
            try: # try to set value
                #2ch 16bit high-precision DAC
                #temp = (ADC_Value[0]>>7)*5.0/0xffff #16-bit integer in hexadecimal with all but the highest bit set
                print ("DAC_A :", device_port.sub_values[topic])
                #print ("DAC_B :",ADDA_DAC8532.DAC_VREF - temp)
                #print ("\33[10A") # for colored print
                DAC.DAC8532_Out_Voltage(ADDA_DAC8532.channel_A, device_port.sub_values[topic])
                device_port.sub_events[topic].clear()
                #DAC.DAC8532_Out_Voltage(ADDA_DAC8532.channel_B, ADDA_DAC8532.DAC_VREF - temp)
            except Exception as e:
                set_err[0] += 1
                err_msg = f"{slave.name}_set_err_{set_err} at {round((time.time()-start),2)}s: " + str(e)
                logging.error(err_msg)
            finally:
                logging.info(f"{slave.name}_set_err: {round((set_err[1] - set_err[0])/(set_err[1]+0.00000000000000001)*100, 2)}%")


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
    _average_interval_lst = []
    # calc average min flow rate by each interval
    try: 
        for interval in range(30, 55, 5):
            _flow_rate_interval_lst = []
            # for each interval, calculate the average flow rate
            for i in range(interval, len(_time_readings), interval):
                # flow rate in [liter/s]
                # 0.1 liter / pulse
                _flow_rate = 60 * 0.1 * (interval-1) / (_time_readings[i-1] - _time_readings[i-interval])
                _flow_rate_interval_lst.append(round(_flow_rate, 2)) 
            _average_flow_rate_interval = round(sum(_flow_rate_interval_lst) / len(_flow_rate_interval_lst), 2)          
            _average_interval_lst.append(_average_flow_rate_interval)
        _average = round(sum(_average_interval_lst) / len(_average_interval_lst), 1)
        _readings = tuple([_sampling_time, _average])
    except:
        _readings = tuple([_sampling_time, 0])
    return _readings
            

@kb_event
@sampling_event(params.sample_time_DFM)
@analyze_decker
def DFM_AOG_data_analyze(start, device_port, slave, **kwargs):
    _time_readings = kwargs.get('_time_readings')
    _sampling_time = round(time.time()-start, 2)
    _average_interval_lst = []
    # calc average min flow rate by each interval 
    try:
        for interval in range(5, 30, 5):
        #for interval in range(30, 55, 5):
            _flow_rate_interval_lst = []
            # for each interval, calculate the average flow rate
            for i in range(interval, len(_time_readings), interval):
                # flow rate in [liter/s]
                # 0.01 liter / pulse
                _flow_rate = 60 * 0.01 * (interval-1) / (_time_readings[i-1] - _time_readings[i-interval])
                _flow_rate_interval_lst.append(round(_flow_rate, 2)) 
            _average_flow_rate_interval = round(sum(_flow_rate_interval_lst) / len(_flow_rate_interval_lst), 2)          
            _average_interval_lst.append(_average_flow_rate_interval)
        _average = round(sum(_average_interval_lst) / len(_average_interval_lst), 1)
        _readings = tuple([_sampling_time, _average])
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
    _arr_readings = np.array(
        [[int(readings[i:i+4],16)/100 for i in range(8,20,4)] 
        + [int(readings[24:28],16)/100] 
        + [int(readings[-12:-8],16)/100] 
        + [(lambda i: ((i[0]*256+i[1]+i[2])*256+i[3])/100)([int(readings[i:i+2],16) for i in range(-20,-12,2)])] 
        for readings in _lst_readings]
        )
    _lst_readings = tuple(np.round(np.sum(_arr_readings, axis=0) / len(_lst_readings), 1))
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
def BRPump_READ_analyze(start, device_port, slave, **kwargs):
    _lst_readings = kwargs.get('_lst_readings')
    _time_readings = kwargs.get('_time_readings')
    _lst_readings = tuple([np.sum(_lst_readings) / len(_lst_readings)])
    _readings = tuple([round(_time_readings,2)]) + _lst_readings
    return _readings
