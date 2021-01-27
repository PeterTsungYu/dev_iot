from crccheck.crc import Crc16Modbus
import numpy as np

# Quick calculation
data = bytearray.fromhex("03 03 00 00 00 01")
crc = Crc16Modbus.calchex(data)
print(crc[-2:] + ' ' + crc[:2])

rtu = '030300000001'
class Slave:     
    def __init__(self, idno, rtu):
        self.id = idno # id number of slave
        self.crc = Crc16Modbus.calchex(bytearray.fromhex(rtu))
        self.rtu = rtu + self.crc[-2:] + self.crc[:2]  # rtu sent by master
        self.lst_readings = {'temp_0':[]} # record readings
        self.time_readings = {'temp_0':[]} # record time
        self.arr_readings = np.array([]) # for all data 


def gen_Slave():
    slave_3 = Slave(3, 
        '030300000001'
        )
    return slave_3

slave_3 = gen_Slave()
print('succeed')
print(slave_3.rtu)