#python packages
import threading
import serial
import RPi.GPIO as GPIO
from crccheck.crc import Crc16Modbus

#custom modules
import Modbus
import params

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

#-----Cls----------------------------------------------------------------
def tohex(value):
    value = int(value)
    hex_value = hex(value)[2:]
    add_zeros = 4 - len(hex_value)
    hex_value = add_zeros * '0' + hex_value
    return hex_value

class device_port:
    def __init__(self, *slaves, name, port):
        self.slaves = slaves # lst of slaves
        self.name = name 
        self.port = port
        self.sub_topics=[]
        self.pub_topics=[]
        self.err_topics=[]
        self.sub_values = {}
        self.sub_events = {}
        self.pub_values = {}
        self.err_values = {}
        self.thread_funcs = []

        for _slave in slaves:
            for topic in _slave.port_topics.sub_topics:
                self.sub_topics.append(topic)
                self.sub_values[topic] = 0
                self.sub_events[topic] = threading.Event()
            for topic in _slave.port_topics.pub_topics:
                self.pub_topics.append(topic)
                self.pub_values[topic] = 0
            for topic in _slave.port_topics.err_topics:
                self.err_topics.append(topic)
                self.err_values[topic] = [0,0] #[err, click_throu]
    
    def serial_funcs(self, start): 
        def thread_func():
            while not params.kb_event.isSet():
                for slave in self.slaves:
                    if slave.kwargs.get('comm_func'):
                        slave.kwargs['comm_func'](start, self, slave)
        
        self.thread_funcs.append(threading.Thread(
                                    name = f'{self.name}_comm',
                                    target=thread_func, 
                                    #args=(,)
                                    )
                                )

    def parallel_funcs(self, start): 
        for slave in self.slaves:
            if slave.kwargs.get('analyze_func'):
                self.thread_funcs.append(
                    threading.Thread(
                        name=f'{slave.name}_analyze',
                        target=slave.kwargs['analyze_func'],
                        args=(start, self, slave,)
                    )
                )


class port_Topics:
    def __init__(self, sub_topics, pub_topics, err_topics):
        self.event = threading.Event()
        self.sub_topics = sub_topics
        self.pub_topics = pub_topics
        self.err_topics = err_topics

class Slave: # Create Slave data store 
    def __init__(self, name, idno, port_topics, **kwargs):
        self.name = name
        self.id = idno # id number of slave
        self.lst_readings = [] # record readings
        self.time_readings = [] # record time
        self.readings = [] # for all data
        self.port_topics = port_topics
        self.kwargs = kwargs # dict of funcs

    def read_rtu(self, *_fields, wait_len):
        self.r_wait_len = wait_len
        # _fields[0]:data_site
        # _fields[1]:value / data_len
        if len(_fields) == 2:
            data_struc = self.id + '03' + _fields[0] + _fields[1]
            crc = Crc16Modbus.calchex(bytearray.fromhex(data_struc))
            self.r_rtu = data_struc + crc[-2:] + crc[:2]
        elif len(_fields) == 1:
            self.r_rtu = _fields[0]

    def write_rtu(self, *_fields):
        self.w_wait_len = 8
        # _fields[0]:data_site
        # _fields[1]:value / data_len
        if len(_fields) == 2:
            _value = tohex(_fields[1])
            data_struc = self.id + '06' + _fields[0] + _value
            crc = Crc16Modbus.calchex(bytearray.fromhex(data_struc))
            self.w_rtu = data_struc + crc[-2:] + crc[:2]
    

#-------------------------RTU & Slave--------------------------------------
# ADAM_TC
# RTU func code 03, PV value site starts at '0000', data_len is 8 ('0008')
ADAM_TC_slave = Slave(
                    name = 'ADAM_TC',
                    idno=ADAM_TC_id,
                    port_topics=port_Topics(sub_topics=[],
                                            pub_topics=[
                                                'TC7', 'TC8', 'TC9', 'TC10', # # TC7(ADAM_TC_0), TC8(ADAM_TC_1), TC9(ADAM_TC_2), TC10(ADAM_TC_3) 
                                                'TC11', 'EVA_out', 'RAD_in', 'RAD_out' # TC11(ADAM_TC_4), EVA_out(ADAM_TC_5), RAD_in(ADAM_TC_6), RAD_out(ADAM_TC_7)
                                            ],
                                            err_topics=[
                                                'ADAM_TC_collect_err', 'ADAM_TC_analyze_err',
                                            ]
                                            ),
                    comm_func=Modbus.Modbus_Comm,
                    analyze_func=Modbus.ADAM_TC_analyze
                    )
ADAM_TC_slave.read_rtu('0000', '0008', wait_len=21)

# GA slave
GA_slave = Slave(
                name = 'GA',
                idno=GA_id,
                port_topics=port_Topics(sub_topics=[],
                                        pub_topics=[
                                            'GA_CO', 'GA_CO2', 'GA_CH4',
                                            'GA_H2', 'GA_N2', 'GA_HEAT'
                                        ],
                                        err_topics=[
                                            'GA_collect_err', 'GA_analyze_err',
                                        ]
                                        ),
                comm_func=Modbus.Modbus_Comm,
                analyze_func=Modbus.GA_data_analyze
                )
GA_slave.read_rtu('11 01 60 8E', wait_len=31)

# Scale slave
Scale_slave = Slave(
                    name = 'Scale',
                    idno=Scale_id,
                    port_topics=port_Topics(sub_topics=[],
                                            pub_topics=[
                                                'Scale'
                                            ],
                                            err_topics=[
                                                'Scale_collect_err', 'Scale_analyze_err',
                                            ]
                                            ),
                    comm_func=Modbus.Scale_data_collect,
                    analyze_func=Modbus.Scale_data_analyze
                    )
Scale_slave.read_rtu(wait_len=0)

# TCHeader Rreading, RTU func code 03, PV value site at '008A', data_len is 1 ('0001')
# TCHeader Writing, RTU func code 06, SV value site at '0000'
# r_wait_len=7,
# w_wait_len=8,
Header_EVA_slave = Slave(
                        name = 'Header_EVA',
                        idno=Header_EVA_id,
                        port_topics=port_Topics(
                                sub_topics=[
                                    'Header_EVA_SV', # Header EVA(Header_EVA_SV)
                                ],
                                pub_topics=[
                                    'Header_EVA_PV', # Header EVA(Header_EVA_PV)
                                ],
                                err_topics=[
                                    'Header_EVA_collect_err', 'Header_EVA_set_err', 'Header_EVA_analyze_err',
                                ]
                                ),
                        comm_func=Modbus.Modbus_Comm,
                        analyze_func=Modbus.TCHeader_analyze
                        )
Header_EVA_slave.read_rtu('008A', '0001', wait_len=7)

Header_BR_slave = Slave(
                        name='Header_BR',
                        idno=Header_BR_id, 
                        port_topics=port_Topics(
                                sub_topics=[
                                    'Header_BR_SV', # Header BR(Header_BR_SV)
                                ],
                                pub_topics=[
                                    'Header_BR_PV', # Header BR(Header_BR_PV)
                                ],
                                err_topics=[
                                    'Header_BR_collect_err', 'Header_BR_set_err', 'Header_BR_analyze_err',
                                ]
                                ),
                        comm_func=Modbus.Modbus_Comm,
                        analyze_func=Modbus.TCHeader_analyze
                        )
Header_BR_slave.read_rtu('008A', '0001', wait_len=7)

# ADAM_SET_slave, RTU func code 03, channel site at '0000-0003', data_len is 4 ('0004')
## ch00:+-10V, ch01:0-5V, ch02:0-5V, ch03:0-5V
# r_wait_len=13, # wait for 7 bytes == 7 Hex numbers
# w_wait_len=8,
ADAM_SET_slave = Slave(
                        name='ADAM_SET',
                        idno=ADAM_SET_id,
                        port_topics=port_Topics(
                                sub_topics=[
                                    'PCB_SET_SV', 'Pump_SET_SV', 'Air_MFC_SET_SV', 'H2_MFC_SET_SV' # PCB(ADAM_SET_SV0), Pump(ADAM_SET_SV1), Air_MFC(ADAM_SET_SV2), H2_MFC(ADAM_SET_SV3)
                                ],
                                pub_topics=[
                                    'PCB_SET_PV', 'Pump_SET_PV', 'Air_MFC_SET_PV', 'H2_MFC_SET_PV', # PCB(ADAM_SET_PV0), Pump(ADAM_SET_PV1), Air_MFC(ADAM_SET_PV2), H2_MFC(ADAM_SET_PV3)
                                ],
                                err_topics=[
                                    'ADAM_SET_collect_err', 'ADAM_SET_set_err', 'ADAM_SET_analyze_err',
                                ]
                                ),
                        comm_func=Modbus.Modbus_Comm,
                        analyze_func=Modbus.ADAM_SET_analyze
                    )
ADAM_SET_slave.read_rtu('0000', '0004', wait_len=13)

# ADAM_READ_slave, RTU func code 03, channel site at '0000-0008', data_len is 8 ('0008')
## ch00:4-20mA, ch01:0-5V, ch04:0-5V, ch05:0-5V, ch06:0-5V
ADAM_READ_slave = Slave(
                        name='ADAM_READ',
                        idno=ADAM_READ_id,
                        port_topics=port_Topics(
                                sub_topics=[],
                                pub_topics=[
                                    'SMC_0_PV', 'SMC_1_PV', 'ADAM_READ_PV2', 'ADAM_READ_PV3', 'ADAM_READ_PV4', 'ADAM_READ_PV5', 'Air_MFC_PV', 'H2_MFC_PV' # ADAM_READ_PV0 (SMC), ADAM_READ_PV1 (SMC), ADAM_READ_PV2, ADAM_READ_PV3, ADAM_READ_PV4(pump), ADAM_READ_PV5(Air_MFC), ADAM_READ_PV6(H2_MFC), ADAM_READ_PV7
                                ],
                                err_topics=[
                                    'ADAM_READ_collect_err', 'ADAM_READ_analyze_err',
                                ]
                                ),
                        comm_func=Modbus.Modbus_Comm,
                        analyze_func=Modbus.ADAM_READ_analyze
                        )
ADAM_READ_slave.read_rtu('0000', '0008', wait_len=21)

# DFMs' slaves
DFM_slave = Slave(
                name='DFM',
                idno=DFM_id,
                port_topics=port_Topics(
                                sub_topics=[],
                                pub_topics=[
                                    'DFM_RichGas',
                                ],
                                err_topics=[
                                    'DFM_collect_err', 'DFM_analyze_err', 
                                ]
                                ),
                analyze_func=Modbus.DFM_data_analyze
                )

DFM_AOG_slave = Slave(
                    name='DFM_AOG',
                    idno=DFM_AOG_id, 
                    port_topics=port_Topics(
                                sub_topics=[],
                                pub_topics=[
                                    'DFM_AOG'
                                ],
                                err_topics=[
                                    'DFM_AOG_collect_err', 'DFM_AOG_analyze_err'
                                ]
                                ),
                    analyze_func=Modbus.DFM_AOG_data_analyze
                    )

print('Slaves are all set')

#-----Port setting----------------------------------------------------------------
RS485_port = device_port(
                        ADAM_TC_slave,
                        name='RS485_port',
                        port=serial.Serial(port=RS485_port_path,
                                            baudrate=19200, 
                                            bytesize=8, 
                                            stopbits=1, 
                                            parity='N'),
                        )

Scale_port = device_port(Scale_slave,
                        name='Scale_port',
                        port=serial.Serial(port=Scale_port_path,
                                            baudrate=9600, 
                                            bytesize=8, 
                                            stopbits=1, 
                                            parity='N'),
                        )

RS232_port = device_port(GA_slave,
                        name='RS232_port',
                        port=serial.Serial(port=RS232_port_path,
                                            baudrate=9600, 
                                            bytesize=8, 
                                            stopbits=1, 
                                            parity='N'),
                        )

Setup_port = device_port(
                        Header_EVA_slave,
                        Header_BR_slave,
                        ADAM_SET_slave,
                        ADAM_READ_slave,
                        name='Setup_port',
                        port=serial.Serial(port=Setup_port_path,
                                            baudrate=115200, 
                                            bytesize=8, 
                                            stopbits=1, 
                                            parity='N'),
                        )

GPIO_port = device_port(DFM_slave,
                        DFM_AOG_slave,
                        name='GPIO_port',
                        port='GPIO',
                        )

lst_ports = [
            #RS485_port,
            #Scale_port, 
            #RS232_port, 
            Setup_port,
            #GPIO_port
            ]

print('Ports are all set')