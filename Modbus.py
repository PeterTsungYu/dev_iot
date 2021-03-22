from crccheck.crc import Crc16Modbus
import numpy as np
import serial
import time
import threading
import re
from paho.mqtt import client as mqtt_client

from pymodbus.device import ModbusDeviceIdentification
from pymodbus.server.asynchronous import StartSerialServer
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer
from pymodbus.server.asynchronous import StopServer

#-------------------------MQTT--------------------------------------
def connect_mqtt(client_id_mqtt, hostname_mqtt='localhost', port_mqtt=1883):
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id_mqtt)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(hostname_mqtt, port_mqtt)
    return client

client_mqtt = connect_mqtt(client_id_mqtt='rpi-001-mqtt')
client_mqtt.loop_start()

topic_ADAM_TC = [
    "/rpi/Reformer_TC_07", "/rpi/Reformer_TC_08", "/rpi/Reformer_TC_09", "/rpi/Reformer_TC_10",
    "/rpi/Reformer_TC_11", "/rpi/Reformer_TC_12", "/rpi/Reformer_TC_13", "/rpi/Reformer_TC_14"]

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


def serverDB_gen(slave_id=0x00):
    register_block = ModbusSequentialDataBlock(0x00, [0x00]*0x17) # each address can hold from a range 0x00 to 0xffff
    store = ModbusSlaveContext(
        #di=ModbusSequentialDataBlock(0, [1]*100),
        #co=ModbusSequentialDataBlock(0, [2]*100),
        hr=register_block, # holding register block, for func = 3, 6, 16
        #ir=ModbusSequentialDataBlock(0, [4]*100),
        zero_mode=True
        )
    context = ModbusServerContext(
        slaves={slave_id:store,}, # collection of slaves; here only slave 6
        single=False
        )
    print("Succeed to generate a server context")
    return context


def run_server(context, port, timeout=1, baudrate=115200, stopbits=1, bytesize=8, parity='N'):
    
    identity = ModbusDeviceIdentification()
    identity.VendorName = 'RPi'
    identity.ProductCode = 'RPi'
    identity.VendorUrl = ''
    identity.ProductName = 'RPi Server'
    identity.ModelName = 'RPi Server'
    identity.MajorMinorRevision = '0.0.1'

    StartSerialServer(
        context, 
        framer=ModbusRtuFramer,
        identity=identity,  
        port=port, 
        timeout=timeout, 
        baudrate=baudrate,
        stopbits=stopbits,
        bytesize=bytesize,
        parity=parity, 
        )

    print("Server is offline")


def RPiserver(kb_event, port, slave):
    while not kb_event.isSet():
        try:
            # '06' is slave 6
            # '03' is func code 
            # 17*2 data entries
            writing = '0603' + hex(34)[2:]
            #print(slave.readings)
            for i in slave.readings:
                i = hex(i)
                if len(i) != 6: # ex. 0x10
                    i = '0'*(6-len(i)) + i[2:]
                else:
                    i = i[2:]
                writing = writing + i
            crc = Crc16Modbus.calchex(bytearray.fromhex(writing)) # ex. bytearray.fromhex('0010'), two-by-two digits in the bytearray
            writing = writing + crc[-2:] + crc[:2]
            #print(writing)

            readings = port.read(port.inWaiting()).hex()
            if slave.rtu in readings: # Rpi protocol, '06 03 0000 0017 042F'
                port.write(bytes.fromhex(writing)) #hex to binary(byte)
                readings = '' 
                print('RPiserver: write')
            port.reset_input_buffer()
        except Exception as e1:
            print ("RPiserver error: " + str(e1))
    port.close()
    print('kill RPiserver')

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
            slave.time_readings.append(round(time.time()-start, 2))
            if port.inWaiting():
                readings = port.read(port.inWaiting()).decode('utf-8')
                readings = [float(s) if s[0] != '-' else -float(s[1:]) for s in re.findall(r'[ \-][ .\d]{7}', readings)]
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
        
        #print(port.inWaiting())
        if port.inWaiting() == wait_data:
            readings = port.read(port.inWaiting()).decode('utf-8')
            #print(readings)
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


def Adam_data_analyze(kb_event, ticker, sample_time, slave,server_DB):
    while not kb_event.isSet():
        if not ticker.wait(sample_time):
            lst_readings = slave.lst_readings
            time_readings = slave.time_readings
            slave.lst_readings = []
            slave.time_readings = []
            try:
                arr_readings = np.array([[int(reading[i-4:i],16) for i in range(10,len(reading)-2,4)] for reading in lst_readings])
                lst_readings = tuple(np.round(1370/65535*(np.sum(arr_readings, axis=0) / len(lst_readings)), 1))
                #print(lst_readings)
                readings = tuple([round(time_readings[-1],2)]) + lst_readings
                #print(readings)
            except Exception as e1:
                readings = tuple([round(time_readings[-1],2)]) + (0,0,0,0,0,0,0,0)
                print ("Adam_data_analyze error: " + str(e1))
            finally:
                print(f'Adam_data_analyze done: {readings}')
                '''
                conn.execute(
                    "INSERT INTO ADAM_TC(Time, TC_0, TC_1, TC_2, TC_3, TC_4, TC_5, TC_6, TC_7) VALUES (?,?,?,?,?,?,?,?,?);", 
                    readings
                    )
                '''
                slave.readings.append(readings)
                # RTU write to master
                # 0x09:1f:TC_0, 0x10:1f:TC_1, 0x11:1f:TC_2, 0x12:1f:TC_3, 0x13:1f:TC_4, 0x14:1f:TC_5, 0x15:1f:TC_6, 0x16:1f:TC_7
                server_DB.readings[9:] = [int(i*10) for i in readings[1:]]
                #server_DB[0x06].setValues(fx=3, address=0x09, values=[int(i*10) for i in readings[1:]])

                # publish via MQTT
                #for i in range(8):
                    #client_mqtt.publish(topic_ADAM_TC[i], readings[i+1])
                    
    print('kill Adam_data_analyze')
    print(f'Final Adam_data_analyze: {slave.readings}')


def DFM_data_analyze(kb_event, ticker, start, sample_time, slave, server_DB):
    while not kb_event.isSet():
        if not ticker.wait(sample_time): # for each sample_time, collect data
            sampling_time = round(time.time()-start, 2)
            try: 
                time_readings = slave.time_readings
                slave.time_readings = []
                average_interval_lst = []
                # calc average min flow rate by each interval 
                for interval in range(30, 55, 5):
                    flow_rate_interval_lst = []
                    # for each interval, calculate the average flow rate
                    for i in range(interval, len(time_readings), interval):
                        # flow rate in [liter/s]
                        # 0.1 liter / pulse
                        flow_rate = 60 * 0.01 * (interval-1) / (time_readings[i-1] - time_readings[i-interval])
                        flow_rate_interval_lst.append(round(flow_rate, 2)) 
                    average_flow_rate_interval = round(sum(flow_rate_interval_lst) / len(flow_rate_interval_lst), 2)          
                    average_interval_lst.append(average_flow_rate_interval)
                    _average = round(sum(average_interval_lst) / len(average_interval_lst), 1)
                readings = tuple(sampling_time, _average)
            except Exception as e1:
                readings = tuple([sampling_time, 0])
                print ("DFM_data_analyze error: " + str(e1))
            finally:
                '''
                conn.execute(
                    "INSERT INTO DFM(Time, FlowRate) VALUES (?,?);", 
                    readings
                    )
                '''
                print(f'DFM_data_analyze done: {readings}')
                slave.readings.append(readings)
                # 0x08:1f:DFM_flowrate
                server_DB.readings[8] = int(readings[-1]*10)
                # server_DB[0x06].setValues(fx=3, address=0x08, values=[int(readings[-1]*10)])
    print('kill DFM_data_analyze')
    print(f'Final DFM_data_analyze: {slave.readings}')
    StopServer()


def Scale_data_analyze(kb_event, ticker, sample_time, slave, server_DB):
    while not kb_event.isSet():
        if not ticker.wait(sample_time):
            lst_readings = slave.lst_readings
            time_readings = slave.time_readings
            slave.lst_readings = []
            slave.time_readings = []
            try:
                lst_readings = [sum(i)/len(i) for i in lst_readings] # average for 1s' data
                lst_readings = round(sum(lst_readings) / len(lst_readings), 2) # average for 1min's data
                readings = tuple([round(time_readings[-1], 2), lst_readings])
            except Exception as ex:
                readings = tuple([round(time_readings[-1], 3), 0])
                print ("Scale_data_analyze error: " + str(ex))
            finally:
                '''
                conn.execute(
                    "INSERT INTO Scale(Time, Weight) VALUES (?,?);", 
                    readings
                    )
                '''
                print(f'Scale_data_analyze done: {readings}')
                slave.readings.append(readings)
                # 0x07:1f:Weight
                server_DB.readings[7] = int(readings[-1]*1000)
                #server_DB[0x06].setValues(fx=3, address=0x07, values=[int(readings[-1]*10)])
    print('kill Scale_data_analyze')
    print(f'Final Scale_data_analyze: {slave.readings}')


def GA_data_analyze(kb_event, ticker, sample_time, slave, server_DB):
    while not kb_event.isSet():
        if not ticker.wait(sample_time):
            lst_readings = slave.lst_readings
            time_readings = slave.time_readings
            slave.lst_readings = []
            slave.time_readings = []
            try:           
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
            except Exception as ex:
                readings = tuple([round(time_readings[-1],2)]) + (0,0,0,0,0,0)
                print ("GA_data_analyze error: " + str(ex))
            finally:
                '''
                conn.execute(
                    "INSERT INTO GA(Time, CO, CO2, CH4, H2, N2, HEAT) VALUES (?,?,?,?,?,?,?);", 
                    readings
                    )
                '''
                print(f'GA_data_analyze done: {readings}')
                slave.readings.append(readings)
                # 0x01:1f:CO, 0x02:1f:CO2, 0x03:1f:CH4, 0x04:1f:H2, 0x05:1f:N2, 0x06:1f:HEAT
                server_DB.readings[1:7] = [int(i*10) for i in readings[1:]]
                #server_DB[0x06].setValues(fx=3, address=0x01, values=[int(i*10) for i in readings[1:]])
    print('kill GA_data_analyze')
    print(f'Final GA_data_analyze: {slave.readings}')


def MFC_data_analyze(kb_event, ticker, sample_time, slave, server_DB):
    #conn = sqlite3.connect(db)
    while not kb_event.isSet():
        if not ticker.wait(sample_time):
            lst_readings = slave.lst_readings
            time_readings = slave.time_readings
            slave.lst_readings = []
            slave.time_readings = []
            try:
                arr_readings = np.array(
                    [
                        (lambda i: [float(s) if s[0] != '-' else -float(s[1:]) for s in re.findall(r'[ +\-][\d.]{6}', i)])(readings) 
                        for readings in lst_readings
                    ]
                    )
                #print(arr_readings)
                lst_readings = tuple(np.round(np.sum(arr_readings, axis=0) / len(lst_readings), 1))
                readings = tuple(time_readings[-1:]) + lst_readings
            except Exception as ex:
                readings = tuple(time_readings[-1:]) + (0,0,0,0,0)
                print ("MFC_data_analyze error: " + str(ex))
            finally:
                '''
                conn.execute(
                    "INSERT INTO MFC(Time, Pressure, Temper, VolFlow, MassFlow, Setpoint) VALUES (?,?,?,?,?,?);", 
                    readings
                    )
                conn.commit()
                '''
                print(f'MFC_data_analyze done: {readings}')
                slave.readings.append(readings)
                # 0x00:1f:MFC_MassFlow
                server_DB.readings[0] = int(readings[-2]*10)
                #server_DB[0x06].setValues(fx=3, address=0x00, values=[int(readings[-2]*10),])
    print('kill MFC_data_analyze')
    print(f'Final MFC_data_analyze: {slave.readings}')
    #conn.close()
