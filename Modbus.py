from crccheck.crc import Crc16Modbus
import numpy as np

class RTU:     
    def __init__(self, idno, func_code, data_site, data_len):
        self.id = idno # id number of slave
        self.data_len = data_len
        data_struc = idno + func_code + data_site + data_len
        crc = Crc16Modbus.calchex(bytearray.fromhex(data_struc))
        self.rtu = data_struc + crc[-2:] + crc[:2]


class Slave:     
    def __init__(self, idno, *rtu):
        self.id = idno # id number of slave
        self.rtu = rtu # tuple of rtu value
        self.tempor = [] # for temporary data
        self.lst_readings = [] # record readings
        self.time_readings = [] # record time
        self.arr_readings = np.array([]) # for all data 
