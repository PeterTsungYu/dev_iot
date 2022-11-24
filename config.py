#python packages
# import threading
import multiprocessing
import functools
import serial
from crccheck.crc import Crc16Modbus
from datetime import datetime
import time

#custom modules
import Modbus
import params
import PIDsim

#-----------------Database----------------------------------------------
db_time = datetime.now().strftime('%Y_%m_%d_%H_%M_SE')
db_table = False

#-----------------Serial port and DeviceID------------------------------
#_port_path = '/dev/ttyUSB'
#RS485_port_path = '/dev/ttyUSB_RS485' # for monitoring (TC from ADAM)
port_path_dict = {}
port_path_dict['Scale_port_path'] = '/dev/ttyUSB_Scale' # for monitoring Scale
port_path_dict['RS232_port_path'] = '/dev/ttyUSB_RS232' # for monitoring GA
port_path_dict['Setup_port_path'] = '/dev/ttyUSB_PC' # for controling (ADAM, TCHeader)
port_path_dict['PID_port'] = 'PID_port'
port_path_dict['Theoretical'] = 'Theoretical_port'

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
theoretical_id = '19'
ADAM_SET_Subber_id   = '20'
Pressure_READ_id  = '21'
BRnozzlePID_id = '22'
AccPressurePID_id = '23'
EVAnozzlePID_id = '24'

#-----GPIO port setting----------------------------------------------------------------
## DFM
# read High as 3.3V
channel_DFM     = 24
channel_DFM_AOG = 23
GPIO_PWM_1      = 26 #GPIO 26 (PWM)
GPIO_EVA_PWM    = 18
GPIO_PWM_2      = 6
#-----Cls----------------------------------------------------------------
# def tohex(value):
def tohex_pad4(value):
    value = int(value)
    hex_value = hex(value)[2:]
    add_zeros = 4 - len(hex_value)
    hex_value = add_zeros * '0' + hex_value
    return hex_value

def tohex_pad2(value):
    value = int(value)
    hex_value = hex(value)[2:]
    add_zeros = 2 - len(hex_value)
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
        # self.thread_funcs = []
        self.comm_ticker = multiprocessing.Event()
        self.analyze_ticker = multiprocessing.Event()

        for _slave in slaves:
            for topic in _slave.port_topics.sub_topics:
                self.sub_topics.append(topic)
                self.sub_values[topic] = multiprocessing.Value('d', 0.0)
                self.sub_events[topic] = multiprocessing.Event()
            for topic in _slave.port_topics.pub_topics:
                self.pub_topics.append(topic)
                self.pub_values[topic] = multiprocessing.Value('d', 0.0)
            for topic in _slave.port_topics.err_topics:
                self.err_topics.append(topic)
                self.err_values[topic] = multiprocessing.Array('i', 3) #[err, click_throu, correct]
                self.recur_count[topic] = multiprocessing.Array('i', 2) #[one_call_recur, total_recur]
    
    def comm_funcs(self, start): 
        self.comm_ticker.set()
        def comm_process():
            while not params.kb_event.is_set():
                self.comm_ticker.wait()
                for slave in self.slaves:
                    if slave.kwargs.get('comm_func'):
                        slave.kwargs['comm_func'](start, self, slave)
                self.analyze_ticker.set()
        multiprocessing.Process(
            name = f'{self.name}_comm',
            target=comm_process, 
            ).start()

    def analyze_funcs(self, start): 
        def analyze_process():
            while not params.kb_event.is_set():
                lst_analyze_funcs = []
                for slave in self.slaves:
                    if slave.kwargs.get('analyze_func'):
                        lst_analyze_funcs.append(
                            multiprocessing.Process(
                                name=f'{slave.name}_analyze',
                                target=slave.kwargs['analyze_func'],
                                args=(start, self, slave,)
                            )
                        )
                time.sleep(params.sample_time)
                self.comm_ticker.clear()
                self.analyze_ticker.wait()
                for process in lst_analyze_funcs:
                    process.start()
                for process in lst_analyze_funcs:
                    process.join()
                self.analyze_ticker.clear()
                self.comm_ticker.set()
        multiprocessing.Process(
            name = f'{self.name}_analyze',
            target=analyze_process, 
            ).start()
    
    def control_funcs(self, start): 
        for slave in self.slaves:
            if slave.kwargs.get('control_func'):
                multiprocessing.Process(
                    name=f'{slave.name}_control',
                    target=slave.kwargs['control_func'],
                    args=(self, slave,)
                ).start()

class port_Topics:
    def __init__(self, sub_topics, pub_topics, err_topics):
        self.event = multiprocessing.Event()
        self.sub_topics = sub_topics
        self.pub_topics = pub_topics
        self.err_topics = err_topics

class Slave: # Create Slave data store 
    def __init__(self, name, idno, port_topics, **kwargs):
        self.name = name
        self.id = idno # id number of slave
        self.lst_readings = multiprocessing.Queue()
        self.time_readings = multiprocessing.Queue()
        if self.name in ['ADAM_TC', 'ADAM_TC_02', 'Scale', 'DFM', 'DFM_AOG']:
            self.size_lst_readings = {'short_lst_readings':params.manager.list(), 'long_lst_readings':params.manager.list()}
            self.size_time_readings = {'short_time_readings':params.manager.list(), 'long_time_readings':params.manager.list()}
        self.port_topics = port_topics
        self.kwargs = kwargs # dict of funcs

    def read_rtu(self, *_fields, wait_len):
        self.r_wait_len = wait_len
        if len(_fields) == 2:
            data_struc = tohex_pad2(self.id) + '03' + _fields[0] + _fields[1]
            crc = Crc16Modbus.calchex(bytearray.fromhex(data_struc))
            self.r_rtu = data_struc + crc[-2:] + crc[:2]
        elif len(_fields) == 1:
            self.r_rtu = _fields[0]

    def write_rtu(self, *_fields):
        if len(_fields) == 2:
            _value = tohex_pad4(_fields[1])
            data_struc = tohex_pad2(self.id) + '06' + _fields[0] + _value
            crc = Crc16Modbus.calchex(bytearray.fromhex(data_struc))
            self.w_rtu = data_struc + crc[-2:] + crc[:2]
        elif len(_fields) == 1:
            if 'MFC' in self.name:
                self.w_rtu = f'\r{self.id}S{_fields[0]}\r\r'
    def PWM_instance(self, mode: str, frequency=1, duty=0):
        if mode == 'software':
            Modbus.PIG.set_mode(self.id, Modbus.pigpio.OUTPUT)
            Modbus.PIG.set_PWM_range(self.id, 100)
            Modbus.PIG.set_PWM_frequency(self.id, 10)
            Modbus.PIG.set_PWM_dutycycle(self.id, 0)

    def control_constructor(self, Kp=0, Ki=0, Kd=0, beta=0, gamma=0, kick=1, tstep=1, MVmax=100, MVmin=0, SP_range=0, SP_increment=3):
        self.controller = PIDsim.PID(name=f'{self.name}_controller')
        self.controller.update_paramater(Kp, Ki, Kd, beta, gamma, kick, tstep, MVmax, MVmin, SP_range, SP_increment)
#-------------------------RTU & Slave--------------------------------------
# ADAM_TC
# RTU func code 03, PV value site starts at '0000', data_len is 8 ('0008')
ADAM_TC_slave = Slave(
                    name = 'ADAM_TC',
                    idno=ADAM_TC_id,
                    port_topics=port_Topics(sub_topics=[],
                                            pub_topics=[
                                                'TC12', 'Exhaust_gas', 'TC9', 'TC10', # # TC7(ADAM_TC_0), TC8(ADAM_TC_1), TC9(ADAM_TC_2), TC10(ADAM_TC_3) 
                                                'TC11', 'TC6', 'Header_EVA_PV', 'RAD_Out', # TC11(ADAM_TC_4), EVA_out(ADAM_TC_5), RAD_in(ADAM_TC_6), RAD_out(ADAM_TC_7)
                                                'TC12_rate', 'Exhaust_gas_rate', 'TC9_rate', 'TC10_rate', 
                                                'TC11_rate', 'TC6_rate', 'Header_EVA_PV_rate', 'RAD_Out_rate' 
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
                                            #   'Scale_byPython'  
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
                                    'PCB_SET_SV', 'W_Pump_SET_SV', 'RF_Pump_SET_SV', 'BR_Pump_SET_SV', #'Nozzle_Pump_SET_SV'#, Pump(ADAM_SET_SV1), Air_MFC(ADAM_SET_SV2), H2_MFC(ADAM_SET_SV3)
                                ],
                                pub_topics=[
                                    'PCB_SET_PV', 'W_Pump_SET_PV', 'RF_Pump_SET_PV', 'BR_Pump_SET_PV',#'Nozzle_Pump_SET_PV'#, Pump(ADAM_SET_PV1), Air_MFC(ADAM_SET_PV2), H2_MFC(ADAM_SET_PV3)
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

ADAM_SET_Subber_slave = Slave(
                        name='ADAM_SET_Subber',
                        idno=ADAM_SET_Subber_id,
                        port_topics=port_Topics(
                                sub_topics=[
                                    'TOM_SET_SV', 'SOL_1_SET_SV', 'KNF_SET_SV', 'SOL_2_SET_SV' # PCB(ADAM_SET_SV0), Pump(ADAM_SET_SV1), Air_MFC(ADAM_SET_SV2), H2_MFC(ADAM_SET_SV3)
                                ],
                                pub_topics=[
                                    'TOM_SET_PV', 'SOL_1_SET_PV', 'KNF_SET_PV', 'SOL_2_SET_PV', # PCB(ADAM_SET_PV0), Pump(ADAM_SET_PV1), Air_MFC(ADAM_SET_PV2), H2_MFC(ADAM_SET_PV3)
                                ],
                                err_topics=[
                                    'ADAM_SET_Subber_collect_err', 'ADAM_SET_Subber_set_err', 'ADAM_SET_Subber_analyze_err',
                                ]
                                ),
                        # timeout = 0.01,
                        # comm_func=Modbus.Modbus_Comm,
                        # analyze_func=Modbus.ADAM_SET_analyze
                    )
# ADAM_SET_Subber_slave.read_rtu('0000', '0004', wait_len=13)
# ADAM_SET_Subber_slave.w_wait_len = 8

# Pressure_READ_slave, RTU func code 03, channel site at '0000-0008', data_len is 8 ('0008')
## ch00:4-20mA, ch01:0-5V, ch04:0-5V, ch05:0-5V, ch06:0-5V
Pressure_READ_slave = Slave(
                        name='Pressure_READ',
                        idno=Pressure_READ_id,
                        port_topics=port_Topics(
                                sub_topics=[],
                                pub_topics=[
                                    'Subber_READ_PV0', 'Subber_READ_PV1', 'Subber_READ_PV2', 'Subber_READ_PV3', 'Subber_READ_PV4',
                                    'Subber_P_BR', 'Subber_P_EVA', 'Subber_P_ACC' # Pressure_READ_PV0 (SMC), Pressure_READ_PV1 (SMC), Pressure_READ_PV2, Pressure_READ_PV3, Pressure_READ_PV4(pump), Pressure_READ_PV5(Air_MFC), Pressure_READ_PV6(H2_MFC), Pressure_READ_PV7
                                ],
                                err_topics=[
                                    'Pressure_READ_collect_err', 'Pressure_READ_analyze_err',
                                ]
                                ),
                        # timeout = 0.01,
                        # comm_func=Modbus.Modbus_Comm,
                        # analyze_func=Modbus.ADAM_READ_analyze
                        )
# Pressure_READ_slave.read_rtu('0000', '0008', wait_len=21)
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
                    # timeout = 0.1,            
                    # comm_func=Modbus.PWM_comm,
                    #analyze_func=Modbus.,
                    )
PWM01_slave.PWM_instance('software')


PWM02_slave = Slave(
                    name='PWM02',
                    idno=GPIO_PWM_2, #GPIO
                    port_topics=port_Topics(
                                sub_topics=[
                                    'PWM02_open_SV', 'PWM02_f_SV', 'PWM02_duty_SV'
                                ],
                                pub_topics=[
                                ],
                                err_topics=[
                                    'PWM02_collect_err', 'PWM02_set_err', 'PWM02_analyze_err'
                                ]
                                ),
                    # timeout = 0.1,            
                    # comm_func=Modbus.PWM_comm,
                    #analyze_func=Modbus.,
                    )
PWM02_slave.PWM_instance('software')

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
                                    'EVA_PWM_set_err',
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
                                # 'LambdaPID_Kp', 'LambdaPID_Ki', 'LambdaPID_Kd', 'LambdaPID_SP_range', 'LambdaPID_SP_increment', 'LambdaPID_gamma', 'LambdaPID_beta','LambdaPID_tstep', 'LambdaPID_kick',
                                'LambdaPID_MVmin', 'LambdaPID_MVmax', 'LambdaPID_PV', 'LambdaPID_SP', 'LambdaPID_mode', 'LambdaPID_setting', 'LambdaPID_woke',
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
                        control_func=Modbus.control,
                        )
LambdaPID_slave.control_constructor(Kp=0.8, Ki=0.3, Kd=0.5, beta=1, gamma=0, kick=1.5, tstep=3, MVmax=180, MVmin=20, SP_range=0, SP_increment=3)

CurrentPID_slave = Slave(
                        name='CurrentPID',
                        idno=currentpid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                # 'CurrentPID_Kp', 'CurrentPID_Ki', 'CurrentPID_Kd', 'CurrentPID_beta', 'CurrentPID_tstep', 'CurrentPID_kick', 'CurrentPID_SP_range', 'CurrentPID_SP_increment', 'CurrentPID_gamma',
                                'CurrentPID_MVmin', 'CurrentPID_MVmax', 'CurrentPID_PV', 'CurrentPID_SP', 'CurrentPID_mode','CurrentPID_setting', 'CurrentPID_woke',
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
                        control_func=Modbus.control,
                        )
CurrentPID_slave.control_constructor(Kp=0.008, Ki=0.001, Kd=0.08, beta=1, gamma=0, kick=1.2, tstep=10, MVmax=5, MVmin=0.17, SP_range=0, SP_increment=0.1)

CatBedPID_slave = Slave(
                        name='CatBedPID',
                        idno=catbedpid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                'CatBedPID_Kp', 'CatBedPID_Ki', 'CatBedPID_Kd', 
                                'CatBedPID_MVmin',  'CatBedPID_MVmax', 'CatBedPID_PV', 'CatBedPID_SP', 'CatBedPID_mode',
                                'CatBedPID_setting', 'CatBedPID_woke', 'CatBedPID_SP_range', 'CatBedPID_SP_increment', 
                                'CatBedPID_beta', 'CatBedPID_tstep', 'CatBedPID_kick', 'CatBedPID_gamma'
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
                        control_func=Modbus.control,
                        )
CatBedPID_slave.control_constructor()
# (Kp=1, Ki=0.003, Kd=0.9, beta=1, gamma=0, kick=5, tstep=10, MVmax=750, MVmin=400, SP_range=0, SP_increment=2)

PCBPID_slave = Slave(
                        name='PCBPID',
                        idno=pcbpid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                'PCBPID_Kp', 'PCBPID_Ki', 'PCBPID_Kd', 'PCBPID_beta', 'PCBPID_tstep', 'PCBPID_kick', 'PCBPID_SP_range', 'PCBPID_SP_increment', 'PCBPID_gamma',
                                'PCBPID_MVmin',  'PCBPID_MVmax', 'PCBPID_PV', 'PCBPID_SP', 'PCBPID_mode', 'PCBPID_setting', 'PCBPID_woke',
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
                        control_func=Modbus.control,
                        )
PCBPID_slave.control_constructor(Kp=5, Ki=2, Kd=5, beta=1, gamma=0, kick=1.5, tstep=1, MVmax=90, MVmin=0, SP_range=0, SP_increment=3)

BurnerPID_slave = Slave(
                        name='BurnerPID',
                        idno=burnerPID_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                'BurnerPID_Kp', 'BurnerPID_Ki', 'BurnerPID_Kd', 'BurnerPID_beta', 'BurnerPID_tstep', 'BurnerPID_kick', 'BurnerPID_SP_range', 'BurnerPID_SP_increment', 'BurnerPID_gamma'
                                'BurnerPID_MVmin', 'BurnerPID_MVmax', 'BurnerPID_PV', 'BurnerPID_SP', 'BurnerPID_mode', 'BurnerPID_setting', 'BurnerPID_woke', 
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
                        control_func=Modbus.control,
                        )
BurnerPID_slave.control_constructor(Kp=0.0003, Ki=0.000003, Kd=0.001, beta=0.5, gamma=0, kick=4, tstep=5, MVmax=0.5, MVmin=0.15, SP_range=0, SP_increment=3)

EVAPID_slave = Slave(
                        name='EVAPID',
                        idno=evapid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                # 'EVAPID_Kp', 'EVAPID_Ki', 'EVAPID_Kd',  'EVAPID_beta', 'EVAPID_tstep', 'EVAPID_kick', 'EVArPID_SP_range', 'EVAPID_SP_increment', 'EVAPID_gamma',
                                'EVAPID_MVmin',  'EVAPID_MVmax', 'EVAPID_PV', 'EVAPID_SP', 'EVAPID_mode', 'EVAPID_setting', 'EVAPID_woke',
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
                        control_func=Modbus.control,
                        )

EVAPID_slave.control_constructor(Kp=1, Ki=0.3, Kd=1, beta=1, gamma=0, kick=1, tstep=1, MVmax=100, MVmin=80, SP_range=0, SP_increment=3)

BRnozzlePID_slave = Slave(
                        name='BRnozzlePID',
                        idno=BRnozzlePID_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                'BRnozzlePID_Kp', 'BRnozzlePID_Ki', 'BRnozzlePID_Kd', 
                                'BRnozzlePID_MVmin', 'BRnozzlePID_MVmax', 'BRnozzlePID_PV', 'BRnozzlePID_SP', 'BRnozzlePID_mode', 'BRnozzlePID_setting', 'BRnozzlePID_woke', 'BRnozzlePID_SP_range','BRnozzlePID_SP_increment',
                                'BRnozzlePID_beta', 'BRnozzlePID_tstep', 'BRnozzlePID_kick','BRnozzlePID_gamma',
                            ],
                            pub_topics=[
                                'BRnozzlePID_MV', 'BRnozzlePID_P', 'BRnozzlePID_I', 'BRnozzlePID_D'
                            ],
                            err_topics=[
                                'BRnozzlePID_collect_err', 'BRnozzlePID_set_err', 'BRnozzlePID_analyze_err'
                            ]
                            ),
                        #comm_func=Modbus.,
                        #analyze_func=Modbus.,
                        # control_func=Modbus.control,
                        )
# CV_06: burner temperture; MV: PCB SP
## burner_PV > burner_SP => PCB down => DirectAction=False
BRnozzlePID_slave.control_constructor()
# BRnozzlePID_slave.control_constructor_fixed(Kp=0.0003, Ki=0.000003, Kd=0.001, beta=0.5, kick=4, tstep=5, MVmax=0.5, MVmin=0.15, SP_range=0, SP_increment=3)

EVAnozzlePID_slave = Slave(
                        name='EVAnozzlePID',
                        idno=EVAnozzlePID_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                'EVAnozzlePID_Kp', 'EVAnozzlePID_Ki', 'EVAnozzlePID_Kd', 
                                'EVAnozzlePID_MVmin', 'EVAnozzlePID_MVmax', 'EVAnozzlePID_PV', 'EVAnozzlePID_SP', 'EVAnozzlePID_mode', 'EVAnozzlePID_setting', 'EVAnozzlePID_woke', 'EVAnozzlePID_SP_range','EVAnozzlePID_SP_increment',
                                'EVAnozzlePID_beta', 'EVAnozzlePID_tstep', 'EVAnozzlePID_kick','EVAnozzlePID_gamma',
                            ],
                            pub_topics=[
                                'EVAnozzlePID_MV', 'EVAnozzlePID_P', 'EVAnozzlePID_I', 'EVAnozzlePID_D'
                            ],
                            err_topics=[
                                'EVAnozzlePID_collect_err', 'EVAnozzlePID_set_err', 'EVAnozzlePID_analyze_err'
                            ]
                            ),
                        #comm_func=Modbus.,
                        #analyze_func=Modbus.,
                        # control_func=Modbus.control,
                        )
# CV_06: burner temperture; MV: PCB SP
## burner_PV > burner_SP => PCB down => DirectAction=False
EVAnozzlePID_slave.control_constructor()
# BRnozzlePID_slave.control_constructor_fixed(Kp=0.0003, Ki=0.000003, Kd=0.001, beta=0.5, kick=4, tstep=5, MVmax=0.5, MVmin=0.15, SP_range=0, SP_increment=3)

AccPressurePID_slave = Slave(
                        name='AccPressurePID',
                        idno=AccPressurePID_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                'AccPressurePID_Kp', 'AccPressurePID_Ki', 'AccPressurePID_Kd', 
                                'AccPressurePID_MVmin', 'AccPressurePID_MVmax', 'AccPressurePID_PV', 'AccPressurePID_SP', 'AccPressurePID_mode', 'AccPressurePID_setting', 'AccPressurePID_woke', 'AccPressurePID_SP_range','AccPressurePID_SP_increment',
                                'AccPressurePID_beta', 'AccPressurePID_tstep', 'AccPressurePID_kick','AccPressurePID_gamma',
                            ],
                            pub_topics=[
                                'AccPressurePID_MV', 'AccPressurePID_P', 'AccPressurePID_I', 'AccPressurePID_D'
                            ],
                            err_topics=[
                                'AccPressurePID_collect_err', 'AccPressurePID_set_err', 'AccPressurePID_analyze_err'
                            ]
                            ),
                        #comm_func=Modbus.,
                        #analyze_func=Modbus.,
                        # control_func=Modbus.control,
                        )
# CV_06: burner temperture; MV: PCB SP
## burner_PV > burner_SP => PCB down => DirectAction=False
AccPressurePID_slave.control_constructor()
# AccPressurePID_slave.control_constructor_fixed(Kp=0.0003, Ki=0.000003, Kd=0.001, beta=0.5, kick=4, tstep=5, MVmax=0.5, MVmin=0.15, SP_range=0, SP_increment=3)

Theoretical_slave = Slave(
                        name='Theoretical',
                        idno=theoretical_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                ],
                            pub_topics=[
                                'Convertion_py', 'Current_py', 
                                'Percentage_of_MeOH', 'Percentage_of_H2O', 'Percentage_of_H2', 'Percentage_of_CO', 'Percentage_of_CO2'
                            ],
                            err_topics=[
                            ]
                            ),
                        #comm_func=Modbus.,
                        #analyze_func=Modbus.,
                        # calculate_func=Modbus.Theoretical,
                        )
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

RS232_port = device_port(Air_MFC_slave,
                        GA_slave,
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
                        # PWM01_slave,
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

Theoretical_port = device_port(
                    Theoretical_slave,
                    name='Theoretical_port',
                    port='Theoretical',
                    )

Subber_port = device_port(
                BRnozzlePID_slave,
                EVAnozzlePID_slave,
                AccPressurePID_slave,
                Pressure_READ_slave,
                PWM01_slave,
                PWM02_slave,
                ADAM_SET_Subber_slave,
                name='Subber_port',
                port='Subber',
                )


lst_ports = [
            # MFC_port,
            Scale_port, 
            RS232_port, 
            Setup_port,
            GPIO_port,
            PID_port,
            Subber_port,
            Theoretical_port, #this port should be last one
            ]

# NodeRed = {}
NodeRed = params.manager.dict()

print('Ports are all set')