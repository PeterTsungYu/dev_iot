#python packages
import threading
import signal
import serial
import RPi.GPIO as GPIO
from crccheck.crc import Crc16Modbus

#custom modules

#-------------------------Global var--------------------------------------
time_out          = 1 # for collecting data
sample_time       = 2 # for analyzing data
sample_time_Scale = 5
sample_time_DFM   = 60

# count down events
ticker = threading.Event() # for analyzing data

#-----------------Serial port and DeviceID------------------------------
#_port_path = '/dev/ttyUSB'
RS485_port_path = '/dev/ttyUSB_RS485' # for monitoring (TC from ADAM)
Scale_port_path = '/dev/ttyUSB_Scale' # for monitoring Scale
RS232_port_path = '/dev/ttyUSB_RS232' # for monitoring GA
Setup_port_path = '/dev/ttyUSB_PC' # for controling (ADAM, TCHeader)

## device ID
Header_EVA_id = '01' # ReformerTP EVA_Header @ Setup_port_path
Header_BR_id  = '02' # ReformerTP BR_Header @ Setup_port_path
ADAM_SET_id   = '03' # ReformerTP ADAM_4024 for setting @ Setup_port_path
ADAM_READ_id  = '04' # ReformerTP ADAM_4017+ for monitoring via oltage and current @ Setup_port_path
ADAM_TC_id    = '03' # ReformerTP ADAM_4018+ for monitoring temp @ RS485_port_path
Scale_id      = '06'
DFM_id        = '07'
DFM_AOG_id    = '08'
GA_id         = '11' # ReformerTP GA for monitoring gas conc. @ RS232_port_path

#-----GPIO port setting----------------------------------------------------------------
## DFM
# read High as 3.3V
channel_DFM     = 18
channel_DFM_AOG = 23
GPIO.setmode(GPIO.BCM)

#-----Piping----------------------------------------------------------------
# def a dict
pipe = {
    'R'
}

#-----Cls----------------------------------------------------------------
class device_port:
    def __init__(self, *slaves, port, port_topics):
        self.port = port
        self.port_topics = port_topics
        self.slaves = slaves
class port_Topics:
    def __init__(self, sub_topics, pub_topics):
        self.event = threading.Event()
        self.sub_topics = sub_topics
        self.pub_topics = pub_topics
        self.sub_values = {i:0 for i in self.sub_topics}
        self.sub_events = {i:threading.Event() for i in self.sub_topics}
        self.pub_values = {i:0 for i in self.pub_topics}

def RTU(idno, func_code, data_site, data_len):
        data_struc = idno + func_code + data_site + data_len
        crc = Crc16Modbus.calchex(bytearray.fromhex(data_struc))
        rtu = data_struc + crc[-2:] + crc[:2]
        return rtu # as a string

class Slave: # Create Slave data store 
    def __init__(self, idno, **rtu):
        self.id = idno # id number of slave
        self.rtu = rtu 
        self.lst_readings = [] # record readings
        self.time_readings = 0 # record time
        self.readings = [] # for all data 

#-------------------------RTU & Slave--------------------------------------
# ADAM_TC
# RTU func code 03, PV value site starts at '0000', data_len is 8 ('0008')
ADAM_TC_slave = Slave(idno=ADAM_TC_id, 
                    rtu=RTU(ADAM_TC_id, '03', '0000', '0008')
                    )

# GA slave
GA_slave = Slave(idno=GA_id, 
                rtu='11 01 60 8E'
                )

# Scale slave
Scale_slave = Slave(idno=Scale_id, 
                    rtu=''
                    )

# TCHeader Rreading, RTU func code 03, PV value site at '008A', data_len is 1 ('0001')
Header_EVA_slave = Slave(idno=Header_EVA_id, 
                        rtu=RTU(Header_EVA_id, '03', '008A', '0001')
                        )

Header_BR_slave = Slave(idno=Header_BR_id, 
                        rtu=RTU(Header_BR_id, '03', '008A', '0001')
                        )

# ADAM_SET_slave, RTU func code 03, channel site at '0000-0003', data_len is 4 ('0004')
## ch00:+-10V, ch01:0-5V, ch02:0-5V, ch03:0-5V
ADAM_SET_slave = Slave(idno=ADAM_SET_id, 
                        rtu=RTU(ADAM_SET_id, '03', '0000', '0004') # only ch0
                    )

# ADAM_READ_slave, RTU func code 03, channel site at '0000-0008', data_len is 8 ('0008')
## ch00:4-20mA, ch01:0-5V, ch04:0-5V, ch05:0-5V, ch06:0-5V
ADAM_READ_slave = Slave(idno=ADAM_READ_id, 
                        rtu=RTU(ADAM_READ_id, '03', '0000', '0008') # only ch0
                        )

# DFMs' slaves
DFM_slave = Slave(idno=DFM_id, 
                rtu=''
                )

DFM_AOG_slave = Slave(idno=DFM_AOG_id, 
                    rtu=''
                    )

print('Slaves are all set')

#-----Data----------------------------------------------------------------
RS485_port_Topics = port_Topics(sub_topics=[],
                                pub_topics=[
                                    'TC7', 'TC8', 'TC9', 'TC10', # # TC7(ADAM_TC_0), TC8(ADAM_TC_1), TC9(ADAM_TC_2), TC10(ADAM_TC_3) 
                                    'TC11', 'EVA_out', 'RAD_in', 'RAD_out' # TC11(ADAM_TC_4), EVA_out(ADAM_TC_5), RAD_in(ADAM_TC_6), RAD_out(ADAM_TC_7)
                                    ])

RS232_port_Topics = port_Topics(sub_topics=[],
                                pub_topics=[
                                    'GA_CO', 'GA_CO2', 'GA_CH4',
                                    'GA_H2', 'GA_N2', 'GA_HEAT'
                                    ])

Scale_port_Topics = port_Topics(sub_topics=[],
                                pub_topics=[
                                    'Scale'
                                    ])

Setup_port_Topics = port_Topics(sub_topics=[
                                    'Header_EVA_SV', 'Header_BR_SV', # Header EVA(Header_EVA_SV), Header BR(Header_BR_SV),
                                    'PCB_SET_SV', 'Pump_SET_SV', 'Air_MFC_SET_SV', 'H2_MFC_SET_SV' # PCB(ADAM_SET_SV0), Pump(ADAM_SET_SV1), Air_MFC(ADAM_SET_SV2), H2_MFC(ADAM_SET_SV3)
                                ],
                                pub_topics=[
                                    'Header_EVA_PV', 'Header_BR_PV', # Header EVA(Header_EVA_PV), Header BR(Header_BR_PV),
                                    'PCB_SET_PV', 'Pump_SET_PV', 'Air_MFC_SET_PV', 'H2_MFC_SET_PV', # PCB(ADAM_SET_PV0), Pump(ADAM_SET_PV1), Air_MFC(ADAM_SET_PV2), H2_MFC(ADAM_SET_PV3)
                                    'SMC_0_PV', 'SMC_1_PV', 'ADAM_READ_PV2', 'ADAM_READ_PV3', 'Pump_PV', 'Air_MFC_PV', 'H2_MFC_PV', 'ADAM_READ_PV7' # ADAM_READ_PV0 (SMC), ADAM_READ_PV1 (SMC), ADAM_READ_PV2, ADAM_READ_PV3, ADAM_READ_PV4(pump), ADAM_READ_PV5(Air_MFC), ADAM_READ_PV6(H2_MFC), ADAM_READ_PV7
                                ])

GPIO_port_Topics = port_Topics(sub_topics=[],
                                pub_topics=[
                                    'DFM_RichGas', 'DFM_AOG'
                                ])

err_Topics = port_Topics(sub_topics=[],
                        pub_topics=[
                            'ADAM_TC_collect_err', 'ADAM_TC_analyze_err',
                            'GA_data_collect_err', 'GA_data_analyze_err',
                            'Scale_collect_err', 'Scale_analyze_err',
                            'Header_EVA_collect_err', 'Header_EVA_set_err', 'Header_EVA_analyze_err',
                            'Header_BR_collect_err', 'Header_BR_set_err', 'Header_BR_analyze_err',
                            'ADAM_SET_collect_err', 'ADAM_SET_set_err', 'ADAM_SET_analyze_err',
                            'ADAM_READ_collect_err', 'ADAM_READ_analyze_err',
                            'DFM_data_collect_err', 'DFM_data_analyze_err', 
                            'DFM_AOG_data_collect_err', 'DFM_AOG_data_analyze_err'
                        ])

print('Data is all set')

#-----Port setting----------------------------------------------------------------
RS485_port = device_port(ADAM_TC_slave,
                        port=serial.Serial(port=RS485_port_path,
                                            baudrate=19200, 
                                            bytesize=8, 
                                            stopbits=1, 
                                            parity='N'),
                        port_topics=RS485_port_Topics,
                        )

Scale_port = device_port(Scale_slave,
                        port=serial.Serial(port=Scale_port_path,
                                            baudrate=9600, 
                                            bytesize=8, 
                                            stopbits=1, 
                                            parity='N'),
                        port_topics=Scale_port_Topics,
                        )

RS232_port = device_port(GA_slave,
                        port=serial.Serial(port=RS232_port_path,
                                            baudrate=9600, 
                                            bytesize=8, 
                                            stopbits=1, 
                                            parity='N'),
                        port_topics=RS232_port_Topics,
                        )

Setup_port = device_port(Header_EVA_slave,
                        Header_BR_slave,
                        ADAM_SET_slave,
                        ADAM_READ_slave,
                        port=serial.Serial(port=Setup_port_path,
                                            baudrate=115200, 
                                            bytesize=8, 
                                            stopbits=1, 
                                            parity='N'),
                        port_topics=Setup_port_Topics,
                        )

GPIO_port = device_port(DFM_slave,
                        DFM_AOG_slave,
                        port='GPIO',
                        port_topics=GPIO_port_Topics,
                        )

print('Ports are all set')

#-----------------Interrupt events------------------------------
# Keyboard interrupt event to kill all the threads (Ctr + C)
kb_event = threading.Event()
def signal_handler(signum, frame):
    kb_event.set()
signal.signal(signal.SIGINT, signal_handler) # Keyboard interrupt to stop the program


def terminate(event): # ask user input to stop the program
    print(event.is_set())
    text = input("Type 'exit' to terminate the program...\n>")
    if text == 'exit':
        event.set()




