# pip3 install crccheck
# pip3 install numpy 
## for numpy, also do this: sudo apt-get install libatlas-base-dev
# pip3 install pyserial

#python packages
import numpy as np
import time
import re
from crccheck.crc import Crc16Modbus

#custom modules
import params

#------------------------------Collect and Analyze func---------------------------------
def Modbus_Read(start, port, slave):
    #while not params.kb_event.isSet():
    try:
        port.write(bytes.fromhex(slave.r_rtu)) #hex to binary(byte) 
        
        slave.time_readings.append(round(time.time()-start, 2))

        time.sleep(params.time_out)

        # look up the buffer for 21 bytes, which is for 8 channels data length
        if port.inWaiting() >= slave.r_wait_len: 
            readings = port.read(slave.r_wait_len).hex() # after reading, the buffer will be clean
            crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
            # check sta, func code, datalen, crc
            if (readings[0:2] == slave.id) and (readings[2:4] == '03') and ((crc[-2:] + crc[:2]) == readings[-4:]):
                slave.lst_readings.append(readings)
                print(f'Read from slave_{slave.name}')
            else:
                port.reset_input_buffer() # reset the buffer if no read
                port.err_values[f'{slave.name}_collect_err'] += 1
                print('XX'*10 + f"{slave.name}_collect_err_{port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: crc validation failed" + 'XX'*10) 
        else: # if data len is less than the wait data
            port.reset_input_buffer() # reset the buffer if no read
            port.err_values[f'{slave.name}_collect_err'] += 1
            print('XX'*10 + f"{slave.name}_collect_err_{port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: data len is less than the wait data" + 'XX'*10) 
    except Exception as e:
        port.err_values[f'{slave.name}_collect_err'] += 1
        print('XX'*10 + f"{slave.name}_collect_err_{port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
    finally:
        pass


def Scale_data_collect(start, port, slave):
    #while not params.kb_event.isSet():
    try:
        slave.time_readings = time.time()-start
        time.sleep(params.time_out) # wait for the data input to the buffer
        if port.inWaiting() > slave.r_wait_len:
            readings = port.read(port.inWaiting()).decode('utf-8')
            readings = [float(s) if s[0] != '-' else -float(s[1:]) for s in re.findall(r'[ \-][ .\d]{7}', readings)]
            slave.lst_readings.append(readings)
            print(f'Read from slave_{slave.name}')
            port.reset_input_buffer() # reset the buffer after each reading process
        else: # if data len is no data
            port.err_values[f'{slave.name}_collect_err'] += 1
            print('XX'*10 + f"{slave.name}_collect_err_{port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: data len is no wait data" + 'XX'*10) 
    except Exception as e:
        port.err_values[f'{slave.name}_collect_err'] += 1
        print('XX'*10 + f"{slave.name}_collect_err_{port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
    finally:
        pass


def Modbus_Comm(start, port, slave):    
    #while not params.kb_event.isSet(): # it is written in the ReadingThreads.py
    for topic in slave.port_topics.sub_topics:
        w_data_site=0
        if not port.sub_events[topic].isSet():
            try: # try to collect
                port.write(bytes.fromhex(slave.rtu)) #hex to binary(byte) 
                
                slave.time_readings = time.time()-start

                time.sleep(params.time_out)

                if port.inWaiting() >= slave.r_wait_len: 
                    readings = port.read(slave.r_wait_len).hex() # after reading, the buffer will be clean
                    crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
                    # check sta, func code, datalen, crc
                    if (readings[0:2] == slave.id) and (readings[2:4] == '03') and ((crc[-2:] + crc[:2]) == readings[-4:]):
                        slave.lst_readings.append(readings)
                        print(f'Read from slave_{slave.name}')
                    else:
                        port.reset_input_buffer() # reset the buffer if no read
                        port.err_values[f'{slave.name}_collect_err'] += 1
                        print('XX'*10 + f"{slave.name}_collect_err_{port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: crc validation failed" + 'XX'*10) 
                else: # if data len is less than the wait data
                    port.reset_input_buffer() # reset the buffer if no read
                    port.err_values[f'{slave.name}_collect_err'] += 1
                    print('XX'*10 + f"{slave.name}_collect_err_{port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: data len is less than the wait data" + 'XX'*10) 
            except Exception as e:
                port.err_values[f'{slave.name}_collect_err'] += 1
                print('XX'*10 + f"{slave.name}_collect_err_{port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
            finally:
                pass

        else:
            try: # try to set value
                port.write(bytes.fromhex(slave.write_rtu(f"{w_data_site:0>4}", port.sub_values[topic]))) #hex to binary(byte) 
                
                time.sleep(params.time_out)

                if port.inWaiting() >= slave.w_wait_len:
                    readings = port.read(slave.w_wait_len).hex() # after reading, the buffer will be clean
                    #print(readings)
                    crc = Crc16Modbus.calchex(bytearray.fromhex(readings[:-4]))
                    # check sta, func code, datalen, crc
                    if (readings[0:2] == slave.id) and (readings[2:4] == '06') and ((crc[-2:] + crc[:2]) == readings[-4:]):
                        print(f'Write to slave_{slave.name}')
                    else:
                        port.reset_input_buffer() # reset the buffer if no read
                        port.err_values[f'{slave.name}_collect_err'] += 1
                        print('XX'*10 + f"{slave.name}_collect_err_{port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: crc validation failed" + 'XX'*10) 
                else: # if data len is less than the wait data
                    port.reset_input_buffer() # reset the buffer if no read
                    port.err_values[f'{slave.name}_collect_err'] += 1
                    print('XX'*10 + f"{slave.name}_collect_err_{port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: data len is less than the wait data" + 'XX'*10) 
            except Exception as e:
                port.err_values[f'{slave.name}_collect_err'] += 1
                print('XX'*10 + f"{slave.name}_collect_err_{port.err_values[f'{slave.name}_collect_err']} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
            finally:
                port.sub_events[topic].clear()
        
        w_data_site += 1



def ADAM_TC_analyze(start, port, slave):
    #while not params.kb_event.isSet():
    if not params.ticker.wait(params.sample_time):
        lst_readings = slave.lst_readings
        time_readings = slave.time_readings
        #print(f'ADAM_TC_analyze: {lst_readings}')
        slave.lst_readings = []
        slave.time_readings = []
        try:
            if len(lst_readings) > 0:
                arr_readings = np.array([[int(reading[i-4:i],16) for i in range(10,len(reading)-2,4)] for reading in lst_readings])
                lst_readings = tuple(np.round(1370/65535*(np.sum(arr_readings, axis=0) / len(lst_readings)), 1))
                #print(lst_readings)
                readings = tuple([round(time_readings[-1],2)]) + lst_readings
                #print(readings)

                # casting
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                print(f"{slave.name}_analyze done: record {readings}")
            else:
                print(f"{slave.name}_analyze record nothing")

        except Exception as e:
            port.err_values[f'{slave.name}_analyze_err'] += 1
            print('XX'*10 + f"{slave.name}_analyze_err_{port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
        finally:
            pass


def ADAM_READ_analyze(start, port, slave):
    #while not params.kb_event.isSet():
    if not params.ticker.wait(params.sample_time):
        lst_readings = slave.lst_readings
        time_readings = slave.time_readings
        #print(f'ADAM_TC_analyze: {lst_readings}')
        slave.lst_readings = []
        slave.time_readings = []
        try:
            if len(lst_readings) > 0:
                arr_readings = np.array([[int(reading[i-4:i],16) for i in range(10,len(reading)-2,4)] for reading in lst_readings])
                lst_readings = tuple(np.round(1370/65535*(np.sum(arr_readings, axis=0) / len(lst_readings)), 1))
                #print(lst_readings)
                readings = tuple([round(time_readings[-1],2)]) + lst_readings
                #print(readings)

                # casting
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                print(f"{slave.name}_analyze done: record {readings}")
            else:
                print(f"{slave.name}_analyze record nothing")

        except Exception as e:
            port.err_values[f'{slave.name}_analyze_err'] += 1
            print('XX'*10 + f"{slave.name}_analyze_err_{port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
        finally:
            pass


def DFM_data_analyze(start, port, slave):
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
                    port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                print(f"{slave.name}_analyze done: record {readings}")
            else:
                print(f"{slave.name}_analyze record nothing")

        except Exception as e:
            port.err_values[f'{slave.name}_analyze_err'] += 1
            print('XX'*10 + f"{slave.name}_analyze_err_{port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
        finally:
            pass


def DFM_AOG_data_analyze(start, port, slave):
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
                    port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                print(f"{slave.name}_analyze done: record {readings}")
            else:
                print(f"{slave.name}_analyze record nothing")

        except Exception as e:
            port.err_values[f'{slave.name}_analyze_err'] += 1
            print('XX'*10 + f"{slave.name}_analyze_err_{port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
        finally:
            pass


def Scale_data_analyze(start, port, slave):
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
                    port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                print(f"{slave.name}_analyze done: record {readings}")
            else:
                print(f"{slave.name}_analyze record nothing")

        except Exception as e:
            port.err_values[f'{slave.name}_analyze_err'] += 1
            print('XX'*10 + f"{slave.name}_analyze_err_{port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
        finally:
            pass


def GA_data_analyze(start, port, slave):
    #while not params.kb_event.isSet():
    if not params.ticker.wait(params.sample_time):
        lst_readings = slave.lst_readings
        time_readings = slave.time_readings
        #print(f'GA_data_analyze: {lst_readings}')
        slave.lst_readings = []
        slave.time_readings = []
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
                    port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                print(f"{slave.name}_analyze done: record {readings}")
            else:
                print(f"{slave.name}_analyze record nothing")

        except Exception as e:
            port.err_values[f'{slave.name}_analyze_err'] += 1
            print('XX'*10 + f"{slave.name}_analyze_err_{port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
        finally:
            pass


def TCHeader_analyze(start, port, slave):
    #while (not params.kb_event.isSet()): 
    if not params.ticker.wait(params.sample_time):
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
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                print(f"{slave.name}_analyze done: record {readings}")
            else:
                print(f"{slave.name}_analyze record nothing")

        except Exception as e:
            port.err_values[f'{slave.name}_analyze_err'] += 1
            print('XX'*10 + f"{slave.name}_analyze_err_{port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
        finally:
            pass


def ADAM_SET_analyze(start, port, slave):
    #while (not params.kb_event.isSet())
    if not params.ticker.wait(params.sample_time):
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
                ind = 1
                for topic in slave.port_topics.pub_topics:    
                    port.pub_values[topic] = readings[ind]
                    ind += 1
                ## to slave data list
                slave.readings.append(readings)
                print(f"{slave.name}_analyze done: record {readings}")
            else:
                print(f"{slave.name}_analyze record nothing")

        except Exception as e:
            port.err_values[f'{slave.name}_analyze_err'] += 1
            print('XX'*10 + f"{slave.name}_analyze_err_{port.err_values[f'{slave.name}_analyze_err']} at {round((time.time()-start),2)}s: " + str(e) + 'XX'*10)
        finally:
            pass
