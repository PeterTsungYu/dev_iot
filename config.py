#python packages
import threading
import serial
import RPi.GPIO as GPIO
from crccheck.crc import Crc16Modbus
from datetime import datetime

#custom modules
import Modbus
import params
import PIDsim

#-----------------Database----------------------------------------------
db_time = datetime.now().strftime('%Y_%m_%d_%H_%M_SE')
db_connection = False

#-----------------Serial port and DeviceID------------------------------
#_port_path = '/dev/ttyUSB'
#RS485_port_path = '/dev/ttyUSB_RS485' # for monitoring (TC from ADAM)
port_path_dict = {}
port_path_dict['Scale_port_path'] = '/dev/ttyUSB_Scale' # for monitoring Scale
port_path_dict['RS232_port_path'] = '/dev/ttyUSB_RS232' # for monitoring GA
port_path_dict['Setup_port_path'] = '/dev/ttyUSB_PC' # for controling (ADAM, TCHeader)
port_path_dict['PID_port'] = 'PID_port'

## device ID
Header_EVA_id = '01' # ReformerTP EVA_Header @ Setup_port_path
Header_BR_id  = '02' # ReformerTP BR_Header @ Setup_port_path
ADAM_SET_id   = '03' # ReformerTP ADAM_4024 for setting @ Setup_port_path
ADAM_READ_id  = '04' # ReformerTP ADAM_4017+ for monitoring via oltage and current @ Setup_port_path
ADAM_TC_id    = '05' # ReformerTP ADAM_4018+ for monitoring temp @ Setup_port_path
Scale_id      = '06' # Scale_port_path
DFM_id        = '07' # GPIO
DFM_AOG_id    = '08' # GPIO
GA_id         = '11' # ReformerTP GA for monitoring gas conc. @ RS232_port_path
Air_MFC_id    = 'A'
H2_MFC_id     = 'B'
lambdapid_id  = '12'
currentpid_id = '13'
catbedpid_id  = '14'
pcbpid_id     = '15'
pumppid_id    = '16'
burnerPID_id  = '17'
evapid_id     = '18'

#-----GPIO port setting----------------------------------------------------------------
## DFM
# read High as 3.3V
channel_DFM     = 24
channel_DFM_AOG = 23
GPIO.setmode(GPIO.BCM)
# read High as 3.3V
GPIO_PWM_1      = 26 #GPIO 26 (PWM)
GPIO_EVA_PWM    = 18
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
        self.recur_count = {}
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
                self.err_values[topic] = [0,0,0] #[err, click_throu, recur]
                self.recur_count[topic] = [0]
    
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
            elif slave.kwargs.get('control_func'):
                self.thread_funcs.append(
                    threading.Thread(
                        name=f'{slave.name}_control',
                        target=slave.kwargs['control_func'],
                        args=(self, slave,)
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
        self.time_readings = [] if 'DFM' not in self.name else {'10_time_readings':[], '60_time_readings':[]} # record time
        if 'Scale' in self.name:
            self.scale_readings = {'10_lst_readings':[], '60_lst_readings':[]}
            self.scale_time_readings = {'10_time_readings':[], '60_time_readings':[]}
        else: # record readings for scale
            pass
        self.readings = [] # for all data
        self.port_topics = port_topics
        self.kwargs = kwargs # dict of funcs
        self.GPIO_instance = None

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
        # _fields[0]:data_site
        # _fields[1]:value / data_len
        if len(_fields) == 2:
            _value = tohex(_fields[1])
            data_struc = self.id + '06' + _fields[0] + _value
            crc = Crc16Modbus.calchex(bytearray.fromhex(data_struc))
            self.w_rtu = data_struc + crc[-2:] + crc[:2]
        elif len(_fields) == 1:
            if 'MFC' in self.name:
                self.w_rtu = f'\r{self.id}S{_fields[0]}\r\r'
    def PWM_instance(self, mode: str, frequency=1.0, duty=0):
        if mode == 'software':
            Modbus.GPIO.setup(self.id, Modbus.GPIO.OUT)
            # print(self.id)
            self.GPIO_instance = Modbus.GPIO.PWM(self.id, frequency)  # channel=12 frequency=50Hz (1 Hz up to few kiloHertz.)
            self.GPIO_instance.start(duty)


    def control_constructor(self):
        self.controller = PIDsim.PID(name=f'{self.name}_controller')

    def control_constructor_testing(self, **kwargs):
        self.controller = PIDsim.PID(name=f'{self.name}_controller')
        self.parameters = kwargs
        self.controller.update_paramater_testing(kwargs)
#-------------------------RTU & Slave--------------------------------------
# ADAM_TC
# RTU func code 03, PV value site starts at '0000', data_len is 8 ('0008')
ADAM_TC_slave = Slave(
                    name = 'ADAM_TC',
                    idno=ADAM_TC_id,
                    port_topics=port_Topics(sub_topics=[],
                                            pub_topics=[
                                                'TC12', 'Exhaust_gas', 'TC9', 'TC10', # # TC7(ADAM_TC_0), TC8(ADAM_TC_1), TC9(ADAM_TC_2), TC10(ADAM_TC_3) 
                                                'TC11', 'TC6', 'Header_EVA_PV', 'RAD_Out' # TC11(ADAM_TC_4), EVA_out(ADAM_TC_5), RAD_in(ADAM_TC_6), RAD_out(ADAM_TC_7)
                                            ],
                                            err_topics=[
                                                'ADAM_TC_collect_err', 'ADAM_TC_analyze_err',
                                            ]
                                            ),
                    comm_func=Modbus.Modbus_Comm,
                    analyze_func=Modbus.ADAM_TC_analyze
                    )
ADAM_TC_slave.read_rtu('0000', '0008', wait_len=21)
ADAM_TC_slave.w_wait_len = 8

# GA slave
GA_slave = Slave(
                name = 'GA',
                idno=GA_id,
                port_topics=port_Topics(sub_topics=[
                                        'H2', 'CO2', 'CO', 'MeOH', 'H2O'
                                        ],
                                        pub_topics=[
                                            'GA_CO', 'GA_CO2', 'GA_CH4',
                                            'GA_H2', 'GA_N2', 'GA_HEAT',
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
                    port_topics=port_Topics(sub_topics=[
                                              'Scale_byPython'  
                                            ],
                                            pub_topics=[
                                                '10_Scale', '60_Scale'
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
                                ],
                                pub_topics=[
                                    'EVA_Out', # Header EVA(Header_EVA_PV)
                                ],
                                err_topics=[
                                    'Header_EVA_collect_err', 'Header_EVA_set_err', 'Header_EVA_analyze_err',
                                ]
                                ),
                        comm_func=Modbus.Modbus_Comm,
                        analyze_func=Modbus.TCHeader_analyze
                        )
Header_EVA_slave.read_rtu('008A', '0001', wait_len=7)
Header_EVA_slave.w_wait_len = 8

Header_EVA_SET_slave = Slave(
                        name = 'Header_EVA_SET',
                        idno=Header_EVA_id,
                        port_topics=port_Topics(
                                sub_topics=[
                                    'Header_EVA_SV', 
                                ],
                                pub_topics=[
                                    'Header_EVA_SET_PV'
                                ],
                                err_topics=[
                                    'Header_EVA_SET_collect_err', 'Header_EVA_SET_set_err', 'Header_EVA_SET_analyze_err',
                                ]
                                ),
                        comm_func=Modbus.Modbus_Comm,
                        analyze_func=Modbus.TCHeader_analyze
                        )
Header_EVA_SET_slave.read_rtu('0000', '0001', wait_len=7)
Header_EVA_SET_slave.w_wait_len = 8

Header_BR_slave = Slave(
                        name='Header_BR',
                        idno=Header_BR_id, 
                        port_topics=port_Topics(
                                sub_topics=[
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
Header_BR_slave.w_wait_len = 8

Header_BR_SET_slave = Slave(
                        name='Header_BR_SET',
                        idno=Header_BR_id, 
                        port_topics=port_Topics(
                                sub_topics=[
                                    'Header_BR_SV', # Header BR(Header_BR_SV)
                                ],
                                pub_topics=[
                                    'Header_BR_SET_PV' # Header BR(Header_BR_PV)
                                ],
                                err_topics=[
                                    'Header_BR_SET_collect_err', 'Header_BR_SET_set_err', 'Header_BR_SET_analyze_err',
                                ]
                                ),
                        comm_func=Modbus.Modbus_Comm,
                        analyze_func=Modbus.TCHeader_analyze
                        )
Header_BR_SET_slave.read_rtu('0000', '0001', wait_len=7)
Header_BR_SET_slave.w_wait_len = 8

# ADAM_SET_slave, RTU func code 03, channel site at '0000-0003', data_len is 4 ('0004')
## ch00:+-10V, ch01:0-5V, ch02:0-5V, ch03:0-5V
# r_wait_len=13, # wait for 7 bytes == 7 Hex numbers
# w_wait_len=8,
ADAM_SET_slave = Slave(
                        name='ADAM_SET',
                        idno=ADAM_SET_id,
                        port_topics=port_Topics(
                                sub_topics=[
                                    'PCB_SET_SV', 'H2_MFC_SET_SV', 'RF_Pump_SET_SV', 'BR_Pump_SET_SV', 'Nozzle_Pump_SET_SV'#, Pump(ADAM_SET_SV1), Air_MFC(ADAM_SET_SV2), H2_MFC(ADAM_SET_SV3)
                                ],
                                pub_topics=[
                                    'PCB_SET_PV', 'H2_MFC_SET_PV', 'RF_Pump_SET_PV', 'BR_Pump_SET_PV','Nozzle_Pump_SET_PV'#, Pump(ADAM_SET_PV1), Air_MFC(ADAM_SET_PV2), H2_MFC(ADAM_SET_PV3)
                                ],
                                err_topics=[
                                    'ADAM_SET_collect_err', 'ADAM_SET_set_err', 'ADAM_SET_analyze_err',
                                ]
                                ),
                        comm_func=Modbus.Modbus_Comm,
                        analyze_func=Modbus.ADAM_SET_analyze
                    )
ADAM_SET_slave.read_rtu('0000', '0004', wait_len=13)
ADAM_SET_slave.w_wait_len = 8

# ADAM_READ_slave, RTU func code 03, channel site at '0000-0008', data_len is 8 ('0008')
## ch00:4-20mA, ch01:0-5V, ch04:0-5V, ch05:0-5V, ch06:0-5V
ADAM_READ_slave = Slave(
                        name='ADAM_READ',
                        idno=ADAM_READ_id,
                        port_topics=port_Topics(
                                sub_topics=[],
                                pub_topics=[
                                    'ADAM_P_Air', 'ADAM_P_H2', 'ADAM_P_N2', 'ADAM_P_NH', 'ADAM_P_Out', 'ADAM_READ_Air', 'H2_MFC_PV', 'ADAM_P_MeMix' # ADAM_READ_PV0 (SMC), ADAM_READ_PV1 (SMC), ADAM_READ_PV2, ADAM_READ_PV3, ADAM_READ_PV4(pump), ADAM_READ_PV5(Air_MFC), ADAM_READ_PV6(H2_MFC), ADAM_READ_PV7
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
                                sub_topics=['DFM_RichGas_1min','current','Convertion',
                                ],
                                pub_topics=[
                                    '10_DFM_RichGas', '60_DFM_RichGas',
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
                                sub_topics=['DFM_AOG_1min','Ratio'
                                ],
                                pub_topics=[
                                    '10_DFM_AOG', '60_DFM_AOG',
                                ],
                                err_topics=[
                                    'DFM_AOG_collect_err', 'DFM_AOG_analyze_err'
                                ]
                                ),
                    analyze_func=Modbus.DFM_data_analyze
                    )

Air_MFC_slave = Slave(
                    name='Air_MFC',
                    idno=Air_MFC_id, 
                    port_topics=port_Topics(
                                sub_topics=[
                                    'Air_MFC_SET_SV','Lambda'
                                ],
                                pub_topics=[
                                    'Air_MFC_P', 'Air_MFC_T', 'Air_MFC_LPM', 'Air_MFC_SLPM', 'Air_MFC_SET_PV',
                                ],
                                err_topics=[
                                    'Air_MFC_collect_err', 'Air_MFC_set_err', 'Air_MFC_analyze_err'
                                ]
                                ),
                    comm_func=Modbus.MFC_Comm,
                    analyze_func=Modbus.Air_MFC_analyze,
                    )
Air_MFC_slave.read_rtu(f'\r{Air_MFC_id}\r\r', wait_len=49)
Air_MFC_slave.w_wait_len = 49

H2_MFC_slave = Slave(
                    name='H2_MFC',
                    idno=H2_MFC_id, 
                    port_topics=port_Topics(
                                sub_topics=[
                                    'H2_MFC_SET_SV',
                                ],
                                pub_topics=[
                                    'H2_MFC_P', 'H2_MFC_T', 'H2_MFC_LPM', 'H2_MFC_SLPM', 'H2_MFC_SET_PV',
                                ],
                                err_topics=[
                                    'H2_MFC_collect_err', 'H2_MFC_set_err', 'H2_MFC_analyze_err'
                                ]
                                ),
                    comm_func=Modbus.MFC_Comm,
                    analyze_func=Modbus.H2_MFC_analyze,
                    )
H2_MFC_slave.read_rtu(f'\r{H2_MFC_id}\r\r', wait_len=49)
H2_MFC_slave.w_wait_len = 49

PWM01_slave = Slave(
                    name='PWM01',
                    idno=GPIO_PWM_1, #GPIO
                    port_topics=port_Topics(
                                sub_topics=[
                                    'PWM01_open_SV', 'PWM01_f_SV', 'PWM01_duty_SV'
                                ],
                                pub_topics=[
                                ],
                                err_topics=[
                                    'PWM01_collect_err', 'PWM01_set_err', 'PWM01_analyze_err'
                                ]
                                ),
                    comm_func=Modbus.PWM_comm,
                    #analyze_func=Modbus.,
                    )
PWM01_slave.PWM_instance('software')

EVA_PWM_slave = Slave(
                    name='EVA_PWM',
                    idno=GPIO_EVA_PWM, #GPIO
                    port_topics=port_Topics(
                                sub_topics=[
                                    'EVA_PWM_open_SV', 'EVA_PWM_f_SV', 'EVA_PWM_duty_SV'
                                ],
                                pub_topics=[
                                ],
                                err_topics=[
                                    'EVA_PWM_collect_err', 'EVA_PWM_set_err', 'EVA_PWM_analyze_err'
                                ]
                                ),
                    comm_func=Modbus.PWM_comm,
                    #analyze_func=Modbus.,
                    )
EVA_PWM_slave.PWM_instance('software')

LambdaPID_slave = Slave(
                        name='LambdaPID',
                        idno=lambdapid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                # 'LambdaPID_Kp', 'LambdaPID_Ki', 'LambdaPID_Kd', 
                                'LambdaPID_MVmin', 'LambdaPID_MVmax', 'LambdaPID_PV', 'LambdaPID_SP', 'LambdaPID_mode', 'LambdaPID_setting', 
                                # 'LambdaPID_beta','LambdaPID_tstep', 'LambdaPID_kick',
                            ],
                            pub_topics=[
                                'LambdaPID_MV', 'LambdaPID_P', 'LambdaPID_I', 'LambdaPID_D'
                            ],
                            err_topics=[
                                'LambdaPID_collect_err', 'LambdaPID_set_err', 'LambdaPID_analyze_err'
                            ]
                            ),
                        #comm_func=Modbus.,
                        #analyze_func=Modbus.,
                        control_func=Modbus.control_testing,
                        )
#CV_01: Lambda value; MV: Air
## lambda_PV > lambda_SP => Air down => DirectAction=False
LambdaPID_slave.control_constructor_testing(Kp=0.8, Ki=0.3, Kd=0.5, beta=1, kick=1.5, tstep=3)

CurrentPID_slave = Slave(
                        name='CurrentPID',
                        idno=currentpid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                # 'CurrentPID_Kp', 'CurrentPID_Ki', 'CurrentPID_Kd', 
                                'CurrentPID_MVmin', 'CurrentPID_MVmax', 'CurrentPID_PV', 'CurrentPID_SP', 'CurrentPID_mode', 'CurrentPID_setting', 
                                # 'CurrentPID_beta', 'CurrentPID_tstep', 'CurrentPID_kick',
                            ],
                            pub_topics=[
                                'CurrentPID_MV', 'CurrentPID_P', 'CurrentPID_I', 'CurrentPID_D'
                            ],
                            err_topics=[
                                'CurrentPID_collect_err', 'CurrentPID_set_err', 'CurrentPID_analyze_err'
                            ]
                            ),
                        #comm_func=Modbus.,
                        #analyze_func=Modbus.,
                        control_func=Modbus.control_testing,
                        )
# CV_02: SetCurrent; MV: RF_Pump flow rate
## current_PV > current_SP => RF_Pump down => DirectAction=False
CurrentPID_slave.control_constructor_testing(Kp=0.01, Ki=0.0005, Kd=0.01, beta=1, kick=10, tstep=5, MVmax=5, MVmin=0.17)

CatBedPID_slave = Slave(
                        name='CatBedPID',
                        idno=catbedpid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                # 'CatBedPID_Kp', 'CatBedPID_Ki', 'CatBedPID_Kd', 
                                'CatBedPID_MVmin',  'CatBedPID_MVmax', 'CatBedPID_PV', 'CatBedPID_SP', 'CatBedPID_mode', 'CatBedPID_setting', 
                                # 'CatBedPID_beta', 'CatBedPID_tstep', 'CatBedPID_kick'
                            ],
                            pub_topics=[
                                'CatBedPID_MV', 'CatBedPID_P', 'CatBedPID_I', 'CatBedPID_D'
                            ],
                            err_topics=[
                                'CatBedPID_collect_err', 'CatBedPID_set_err', 'CatBedPID_analyze_err'
                            ]
                            ),
                        #comm_func=Modbus.,
                        #analyze_func=Modbus.,
                        control_func=Modbus.control_testing,
                        )
# CV_03: CatBed TC; MV: Fuel to BR
## CatBed_PV > CatBed_SP => BR_Fuel down => DirectAction=False
CatBedPID_slave.control_constructor_testing(Kp=1, Ki=0.003, Kd=8, beta=0, kick=1, tstep=15, MVmax=750, MVmin=450)

PCBPID_slave = Slave(
                        name='PCBPID',
                        idno=pcbpid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                # 'PCBPID_Kp', 'PCBPID_Ki', 'PCBPID_Kd', 
                                'PCBPID_MVmin',  'PCBPID_MVmax', 'PCBPID_PV', 'PCBPID_SP', 'PCBPID_mode', 'PCBPID_setting', 
                                # 'PCBPID_beta', 'PCBPID_tstep', 'PCBPID_kick'
                            ],
                            pub_topics=[
                                'PCBPID_MV', 'PCBPID_P', 'PCBPID_I', 'PCBPID_D'
                            ],
                            err_topics=[
                                'PCBPID_collect_err', 'PCBPID_set_err', 'PCBPID_analyze_err'
                            ]
                            ),
                        #comm_func=Modbus.,
                        #analyze_func=Modbus.,
                        control_func=Modbus.control_testing,
                        )
# CV_04: PCB %; MV: ratio of AOG
## PCBP_PV > PCBP_SP => AOG down => DirectAction=False
PCBPID_slave.control_constructor_testing(Kp=5, Ki=4, Kd=15, beta=1, kick=1.5, tstep=1)

PumpPID_slave = Slave(
                        name='PumpPID',
                        idno=pumppid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                'PumpPID_Kp', 'PumpPID_Ki', 'PumpPID_Kd', 'PumpPID_MVmin',  'PumpPID_MVmax', 
                                'PumpPID_PV', 'PumpPID_SP', 'PumpPID_mode', 'PumpPID_setting', 'PumpPID_beta',
                                'PumpPID_step', 'PumpPID_kick'
                            ],
                            pub_topics=[
                                'PumpPID_MV', 'PumpPID_P', 'PumpPID_I', 'PumpPID_D'
                            ],
                            err_topics=[
                                'PumpPID_collect_err', 'PumpPID_set_err', 'PumpPID_analyze_err'
                            ]
                            ),
                        #comm_func=Modbus.,
                        #analyze_func=Modbus.,
                        control_func=Modbus.control,
                        )
# CV_05: scale; MV: fuel to reformer
## PumpP_PV > PumpP_SP => RF_fuel down => DirectAction=False
PumpPID_slave.control_constructor()

BurnerPID_slave = Slave(
                        name='BurnerPID',
                        idno=burnerPID_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                # 'BurnerPID_Kp', 'BurnerPID_Ki', 'BurnerPID_Kd', 
                                'BurnerPID_MVmin',  'BurnerPID_MVmax', 'BurnerPID_PV', 'BurnerPID_SP', 'BurnerPID_mode', 'BurnerPID_setting', 
                                # 'BurnerPID_beta', 'BurnerPID_tstep', 'BurnerPID_kick'
                            ],
                            pub_topics=[
                                'BurnerPID_MV', 'BurnerPID_P', 'BurnerPID_I', 'BurnerPID_D'
                            ],
                            err_topics=[
                                'BurnerPID_collect_err', 'BurnerPID_set_err', 'BurnerPID_analyze_err'
                            ]
                            ),
                        #comm_func=Modbus.,
                        #analyze_func=Modbus.,
                        control_func=Modbus.control_testing,
                        )
# CV_06: burner temperture; MV: PCB SP
## burner_PV > burner_SP => PCB down => DirectAction=False

BurnerPID_slave.control_constructor_testing(Kp=0.0003, Ki=0.000005, Kd=0.01, beta=0.1, kick=1.2, tstep=15, MVmax=0.5, MVmin=0.15)

EVAPID_slave = Slave(
                        name='EVAPID',
                        idno=evapid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                # 'EVAPID_Kp', 'EVAPID_Ki', 'EVAPID_Kd', 
                                'EVAPID_MVmin',  'EVAPID_MVmax', 'EVAPID_PV', 'EVAPID_SP', 'EVAPID_mode', 'EVAPID_setting', 
                                # 'EVAPID_beta', 'EVAPID_tstep', 'EVAPID_kick'
                            ],
                            pub_topics=[
                                'EVAPID_MV', 'EVAPID_P', 'EVAPID_I', 'EVAPID_D'
                            ],
                            err_topics=[
                                'EVAPID_collect_err', 'EVAPID_set_err', 'EVAPID_analyze_err'
                            ]
                            ),
                        #comm_func=Modbus.,
                        #analyze_func=Modbus.,
                        control_func=Modbus.control_testing,
                        )

EVAPID_slave.control_constructor_testing(Kp=1, Ki=0.03, Kd=1, beta=1, kick=1, tstep=1)
print('Slaves are all set')

#-----Port setting----------------------------------------------------------------
Scale_port = device_port(Scale_slave,
                        name='Scale_port',
                        port=serial.Serial(port=port_path_dict['Scale_port_path'],
                                            baudrate=9600, 
                                            bytesize=8, 
                                            stopbits=1, 
                                            parity='N'),
                        )

RS232_port = device_port(GA_slave,
                        Air_MFC_slave,
                        #H2_MFC_slave,
                        name='RS232_port',
                        port=serial.Serial(port=port_path_dict['RS232_port_path'],
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
                        Header_EVA_SET_slave,
                        Header_BR_SET_slave,
                        ADAM_TC_slave,
                        name='Setup_port',
                        port=serial.Serial(port=port_path_dict['Setup_port_path'],
                                            baudrate=57600, 
                                            bytesize=8, 
                                            stopbits=1, 
                                            parity='N'),
                        )

GPIO_port = device_port(DFM_slave,
                        DFM_AOG_slave,
                        PWM01_slave,
                        EVA_PWM_slave,
                        name='GPIO_port',
                        port='GPIO',
                        )


PID_port = device_port(
                    LambdaPID_slave,
                    CurrentPID_slave,
                    BurnerPID_slave,
                    PCBPID_slave,
                    CatBedPID_slave,
                    EVAPID_slave,
                    name='PID_port',
                    port='PID',
                    )


lst_ports = [
            # MFC_port,
            Scale_port, 
            RS232_port, 
            Setup_port,
            GPIO_port,
            PID_port,
            ]

NodeRed = {}

print('Ports are all set')