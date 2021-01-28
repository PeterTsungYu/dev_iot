from crccheck.crc import Crc16Modbus
import numpy as np
import serial
import time

class RTU:     
    def __init__(self, idno, func_code, data_site, data_len):
        self.id = idno # id number of slave
        self.data_len = data_len
        data_struc = idno + func_code + data_site + data_len
        crc = Crc16Modbus.calchex(bytearray.fromhex(data_struc))
        self.rtu = data_struc + crc[-2:] + crc[:2]


class Slave:     
    def __init__(self, idno, rtu):
        self.id = idno # id number of slave
        self.rtu = rtu # tuple of rtu value
        self.lst_readings = [] # record readings
        self.time_readings = [] # record time
        self.readings = [] # for all data 


def gen_Slave(RTU):
    slave = Slave(RTU.id, RTU.rtu)
    return slave


def Adam_data_collect(port, slave, start, time_out, wait_data):
    #while True:
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
        print ("communicating error " + str(e1))
    
    # if event.is_set():
    ## break


def Adam_data_analyze(slave):
    # while not ticker.wait(5):
    lst_readings = slave.lst_readings
    time_readings = slave.time_readings
    slave.lst_readings = []
    slave.time_readings = []

    arr_readings = np.array([[int(reading[i-4:i],16) for i in range(10,len(reading)-2,4)] for reading in lst_readings])
    lst_readings = list(np.sum(arr_readings, axis=0) / len(lst_readings))

    readings = []
    readings.append(time_readings[-1])
    readings.append(lst_readings)
    slave.readings.append(readings)

    
def terminate(event):
    print(event.is_set())
    text = input("Type 'exit' to terminate the program...\n>")
    if text == 'exit':
        event.set()