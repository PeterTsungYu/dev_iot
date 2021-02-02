from crccheck.crc import Crc16Modbus
import numpy as np
import serial
import time
import threading

# RTU
#------------------------------------------------------------------
class RTU:     
    def __init__(self, idno, func_code, data_site, data_len):
        self.id = idno # id number of slave
        self.data_len = data_len
        data_struc = idno + func_code + data_site + data_len
        crc = Crc16Modbus.calchex(bytearray.fromhex(data_struc))
        self.rtu = data_struc + crc[-2:] + crc[:2]


class Slave:     
    def __init__(self, idno='', rtu=''):
        self.id = idno # id number of slave
        self.rtu = rtu # tuple of rtu value
        self.lst_readings = [] # record readings
        self.time_readings = [] # record time
        self.readings = [] # for all data 


def gen_Slave(RTU): # deprecated 
    slave = Slave(RTU.id, RTU.rtu)
    return slave
#------------------------------------------------------------------


# threading
#------------------------------------------------------------------
class SlaveThread(threading.Thread): # deprecated 
    def __init__(self, name='SlaveThread'):
        threading.Thread.__init__(self, name=name)
        #self._kill = threading.Event()
        self._sleepperiod = 1
    '''def run(self):
        while not self._kill.isSet(): # inspect the event till it is set
            self._kill.wait(self._sleepperiod) # inspect every period
        print('end')'''
    '''def join(self, timeout=None):
        self._stopevent.set(  ) # stop the tread by join method
        threading.Thread.join(self, timeout)'''

def terminate(event): # ask user input to stop the program
    print(event.is_set())
    text = input("Type 'exit' to terminate the program...\n>")
    if text == 'exit':
        event.set()
#------------------------------------------------------------------


# func
#------------------------------------------------------------------
def Adam_data_collect(event, port, slave, start, time_out, wait_data):
    while not event.isSet():
        try:
            port.write(bytes.fromhex(slave.rtu)) #hex to binary(byte) 
            slave.time_readings.append(time.time()-start)

            time.sleep(time_out)

            # look up the buffer for 21 bytes, which is for 8 channels data length
            if port.inWaiting() == wait_data: 
                readings = port.read(wait_data).hex()
                slave.lst_readings.append(readings)
            else: # if data is not correct, return as None
                slave.lst_readings.append(None)
                port.reset_input_buffer() 
            print('collect done')
        except Exception as e1:
            print ("Adam_data_collect error " + str(e1))
        # if event.is_set():
        ## break
    port.close()
    print('kill Adam_data_collect')


def Adam_data_analyze(event, ticker, slave):
    while not event.isSet():
        try:
            if not ticker.wait(5):
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
                print('analyze done')
        except Exception as e1:
            print ("Adam_data_analyze error " + str(e1))
    print('kill Adam_data_analyze')
    print(slave.lst_readings)
    print(slave.time_readings)
    print(slave.readings)
