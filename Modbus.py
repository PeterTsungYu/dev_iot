from crccheck.crc import Crc16Modbus
import numpy as np
import serial
import time
import threading
import re

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
        self.rtu = rtu # tuple of rtu value
        self.lst_readings = [] # record readings
        self.time_readings = [] # record time
        self.readings = [] # for all data 

'''
def gen_Slave(RTU): # deprecated 
    slave = Slave(RTU.id, RTU.rtu)
    return slave
'''

#--------------------------Threading--------------------------------
'''
class SlaveThread(threading.Thread): # deprecated 
    def __init__(self, name='SlaveThread'):
        threading.Thread.__init__(self, name=name)
        self._kill = threading.Event()
        self._sleepperiod = 1
    def run(self):
        while not self._kill.isSet(): # inspect the event till it is set
            self._kill.wait(self._sleepperiod) # inspect every period
        print('end')
    def join(self, timeout=None):
        self._stopevent.set(  ) # stop the tread by join method
        threading.Thread.join(self, timeout)
'''

def terminate(event): # ask user input to stop the program
    print(event.is_set())
    text = input("Type 'exit' to terminate the program...\n>")
    if text == 'exit':
        event.set()

#------------------------------func---------------------------------
def Adam_data_collect(kb_event, port, slave, start, time_out, wait_data):
    while not kb_event.isSet():
        try:
            port.write(bytes.fromhex(slave.rtu)) #hex to binary(byte) 
            slave.time_readings.append(round(time.time()-start, 2))

            time.sleep(time_out)

            # look up the buffer for 21 bytes, which is for 8 channels data length
            if port.inWaiting() == wait_data: 
                readings = port.read(wait_data).hex()
                #print(readings)
                slave.lst_readings.append(readings)
            else: # if data is not correct, return as None
                slave.lst_readings.append(None)
                port.reset_input_buffer() 
        except Exception as e1:
            print ("Adam_data_collect error: " + str(e1))
        finally:
            port.reset_input_buffer() # reset the buffer after each reading process
            print('Adam_data_collect: done')
    port.close()
    print('kill Adam_data_collect')


def Scale_data_collect(kb_event, port, slave, start, time_out):
    while not kb_event.isSet():
        try:
            time.sleep(time_out) # wait for the data input to the buffer
            if port.inWaiting():
                readings = port.read(port.inWaiting()).decode('utf-8')
                slave.time_readings.append(round(time.time()-start, 2))
                readings = [float(s) if s[0] != '-' else -float(s[1:]) for s in re.findall(r'[ \-][ \d]{5}\.\d', readings)]
                slave.lst_readings.append(readings) 
        except Exception as e1:
            print ("Scale_data_collect error: " + str(e1))
        finally:
            port.reset_input_buffer() # reset the buffer after each reading process
            print('Scale_data_collect done')
    port.close()
    print('kill Scale_data_collect')


def MFC_data_collect(port, slave, start, time_out, wait_data):
    #while not kb_event.isSet():
    try:
        port.write(bytes(slave.rtu, 'utf-8')) #string to binary(byte) 
        slave.time_readings.append(round(time.time()-start, 2))

        time.sleep(time_out)
        
        print(port.inWaiting())
        if port.inWaiting() == wait_data:
            readings = port.read(port.inWaiting()).decode('utf-8')
            print(readings)
            slave.lst_readings.append(readings)
    except Exception as e1:
        print ("MFC_data_collect error: " + str(e1))
    finally:
        port.reset_input_buffer() # reset the buffer after each reading process
        print('MFC_data_collect done')
    #port.close()
    #print('kill MFC_data_collect')


def GA_data_collect(port, slave, start, time_out, wait_data):
    #while not kb_event.isSet():
    try:
        port.write(bytes.fromhex(slave.rtu)) #hex to binary(byte) 
        slave.time_readings.append(time.time()-start)

        time.sleep(time_out)

        #print(port.inWaiting())
        if port.inWaiting() == wait_data: 
            readings = port.read(wait_data).hex()
            #print(len(readings))
            slave.lst_readings.append(readings)
        else: # if data is not correct, return as None
            slave.lst_readings.append(None)
            port.reset_input_buffer() 
    except Exception as e1:
        print ("GA_data_collect error: " + str(e1))
    finally:
        port.reset_input_buffer() # reset the buffer after each reading process
        print('GA_data_collect done')
    #port.close()
    #print('kill MFC_data_collect')


def Adam_data_analyze(kb_event, ticker, sample_time, slave):
    while not kb_event.isSet():
        try:
            if not ticker.wait(sample_time):
                lst_readings = slave.lst_readings
                time_readings = slave.time_readings
                slave.lst_readings = []
                slave.time_readings = []

                arr_readings = np.array([[int(reading[i-4:i],16) for i in range(10,len(reading)-2,4)] for reading in lst_readings])
                lst_readings = list(np.round(1370/65535*(np.sum(arr_readings, axis=0) / len(lst_readings)), 2))

                readings = []
                readings.append(round(time_readings[-1],2))
                readings.append(lst_readings)
                slave.readings.append(readings)
                print(f'Adam_data_analyze done: {readings}')
        except Exception as e1:
            print ("Adam_data_analyze error: " + str(e1))
    print('kill Adam_data_analyze')
    print(f'Final Adam_data_analyze: {slave.readings}')


def DFM_data_analyze(kb_event, ticker, sample_time, slave):
    while not kb_event.isSet():
        try:
            if not ticker.wait(sample_time): # for each sample_time, collect data 
                time_readings = slave.time_readings
                slave.time_readings = []

                average_interval_lst = []
                # calc average min flow rate by each interval 
                for interval in range(30, 55, 5):
                    flow_rate_interval_lst = []
                    # for each interval, calculate the average flow rate
                    for i in range(interval, len(time_readings), interval):
                        try:
                            # flow rate in [liter/s]
                            # 0.1 liter / pulse
                            flow_rate = 60 * 0.01 * (interval-1) / (time_readings[i-1] - time_readings[i-interval])
                            flow_rate_interval_lst.append(round(flow_rate, 2)) 
                        except Exception as ex:
                            print("DFM_data_analyze internal loop Error: " + str(ex))
                    average_flow_rate_interval = round(sum(flow_rate_interval_lst) / len(flow_rate_interval_lst), 2)          
                    average_interval_lst.append(average_flow_rate_interval)
                    _average = round(sum(average_interval_lst) / len(average_interval_lst), 2)
                slave.readings.append(_average)
                print(f'DFM_data_analyze done: {_average}')
        except Exception as e1:
            print ("DFM_data_analyze error: " + str(e1))
    print('kill DFM_data_analyze')
    print(f'Final DFM_data_analyze: {slave.readings}')


def Scale_data_analyze(kb_event, ticker, sample_time, slave):
    while not kb_event.isSet():
        try:
            if not ticker.wait(sample_time):
                lst_readings = slave.lst_readings
                time_readings = slave.time_readings
                slave.lst_readings = []
                slave.time_readings = []

                lst_readings = [sum(i)/len(i) for i in lst_readings] # average for 1s' data
                lst_readings = round(sum(lst_readings) / len(lst_readings), 1) # average for 1min's data

                readings = []
                readings.append(time_readings[-1])
                readings.append(lst_readings)
                slave.readings.append(readings)
                print(f'Scale_data_analyze done: {readings}')
        except Exception as ex:
            print ("Scale_data_analyze error: " + str(ex))
    print('kill Scale_data_analyze')
    print(f'Final Scale_data_analyze: {slave.readings}')


def GA_data_analyze(kb_event, ticker, sample_time, slave):
    while not kb_event.isSet():
        try:
            if not ticker.wait(sample_time):
                lst_readings = slave.lst_readings
                time_readings = slave.time_readings
                slave.lst_readings = []
                slave.time_readings = []

                arr_readings = np.array(
                    [[int(readings[i:i+4],16)/100 for i in range(8,20,4)] 
                    + [int(readings[24:28],16)/100] 
                    + [int(readings[-12:-8],16)/100] 
                    + [(lambda i: ((i[0]*256+i[1]+i[2])*256+i[3])/100)([int(readings[i:i+2],16) for i in range(-20,-12,2)])] 
                    for readings in lst_readings]
                    )
                #print(arr_readings)
                lst_readings = list(np.round(np.sum(arr_readings, axis=0) / len(lst_readings), 2))

                readings = []
                readings.append(time_readings[-1])
                readings.append(lst_readings)
                slave.readings.append(readings)
                print(f'GA_data_analyze done: {readings}')
        except Exception as ex:
            print ("GA_data_analyze error: " + str(ex))
    print('kill GA_data_analyze')
    print(f'Final GA_data_analyze: {slave.readings}')


def MFC_data_analyze(kb_event, ticker, sample_time, slave):
    while not kb_event.isSet():
        try:
            if not ticker.wait(sample_time):
                lst_readings = slave.lst_readings
                time_readings = slave.time_readings
                slave.lst_readings = []
                slave.time_readings = []

                arr_readings = np.array(
                    [
                        (lambda i: [float(s) if s[0] != '-' else -float(s[1:]) for s in re.findall(r'[ +\-][\d.]{6}', i)])(readings) 
                        for readings in lst_readings
                    ]
                    )
                #print(arr_readings)
                lst_readings = tuple(np.round(np.sum(arr_readings, axis=0) / len(lst_readings), 2))
                readings = tuple(time_readings[-1:]) + lst_readings
                '''
                conn.execute(
                    "INSERT INTO MFC(Time, Pressure, Temper, VolFlow, MassFlow, Setpoint) VALUES (?,?,?,?,?,?);", 
                    readings
                    )
                '''
                slave.readings.append(readings)
                print(f'MFC_data_analyze done: {readings}')
        except Exception as ex:
            print ("MFC_data_analyze error: " + str(ex))
    print('kill MFC_data_analyze')
    print(f'Final MFC_data_analyze: {slave.readings}')