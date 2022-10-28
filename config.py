#python packages
import threading
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
db_time = datetime.now().strftime('%Y_%m_%d_%H_%M_15kW')
db_connection = False

#-----------------Serial port and DeviceID------------------------------
_port_path = '/dev/ttyUSB'
# MFC_port_path = '/dev/ttyUSB_RS485' # for monitoring MFC (rasp-001_MFC branch)
Scale_port_path = '/dev/ttyUSB_Scale' # for monitoring Scale
RS232_port_path = '/dev/ttyUSB_RS232' # for monitoring GA
Setup_port_path = '/dev/ttyUSB_PC' # for controling (ADAM, TCHeader)
PID_port = 'PID_port'

## device ID
Header_EVA_id = '01' # ReformerTP EVA_Header @ Setup_port_path
Header_BR_id  = '02' # ReformerTP BR_Header @ Setup_port_path
ADAM_SET_id   = '03' # ReformerTP ADAM_4024 for setting @ Setup_port_path
ADAM_READ_id  = '04' # ReformerTP ADAM_4017+ for monitoring via oltage and current @ Setup_port_path
ADAM_TC_id    = '05' # ReformerTP ADAM_4018+ for monitoring temp @ RS485_port_path
Scale_id      = '06'
DFM_id        = '07'
DFM_AOG_id    = '08'
WatchDog_id   = '09'
Relay01_id    = '10' # control Relay for Lambda sensor and Glow Plug
Relay02_id    = '11' 
Air_MFC_id    = 'A'
H2_MFC_id     = 'B'
lambdapid_id  = '12'
currentpid_id = '13'
catbedpid_id  = '14'
pcbpid_id     = '15'
pumppid_id    = '16'
GA_id         = '17' # ReformerTP GA for monitoring gas conc. @ RS232_port_path
burnerPID_id  = '18'
evapid_id     = '19'
#-----GPIO port setting----------------------------------------------------------------
## DFM
# read High as 3.3V
channel_DFM     = 24
channel_DFM_AOG = 25
channel_Relay01_IN1     = 22
channel_Relay01_IN2     = 23
GPIO_PWM_1              = 26 #GPIO 26 (PWM0)    # BR
GPIO_PWM_2              = 6 #GPIO 6 (PWM1)    # SR
GPIO_TTL                = 5
GPIO_MOS                = 21
#-----Cls----------------------------------------------------------------
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
        self.comm_ticker = multiprocessing.Event()
        self.analyze_ticker = multiprocessing.Event()
        # self.broken_slave_names = params.manager.list()


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
                #b =  time.time()
                self.comm_ticker.wait()
                for slave in self.slaves:
                    # if slave.name not in self.broken_slave_names:
                    if slave.kwargs.get('comm_func'):
                        slave.kwargs['comm_func'](start, self, slave)
                self.analyze_ticker.set()
                        # print(slave.name)
                #print(time.time() - b)
        multiprocessing.Process(
            name = f'{self.name}_comm',
            target=comm_process, 
            #args=(,)
            ).start()

    def analyze_funcs(self, start): 
        def analyze_process():
            while not params.kb_event.is_set():
                lst_analyze_funcs = []
                for slave in self.slaves:
                    # if slave.name not in self.broken_slave_names:
                    if slave.kwargs.get('analyze_func'):
                        lst_analyze_funcs.append(
                            multiprocessing.Process(
                                name=f'{slave.name}_analyze',
                                target=slave.kwargs['analyze_func'],
                                args=(start, self, slave,)
                            )
                        )
                time.sleep(1)
                self.comm_ticker.clear()
                self.analyze_ticker.wait()
                # t = time.time()
                for process in lst_analyze_funcs:
                    process.start()
                for process in lst_analyze_funcs:
                    process.join()
                self.analyze_ticker.clear()
                self.comm_ticker.set()
        multiprocessing.Process(
            name = f'{self.name}_analyze',
            target=analyze_process, 
            #args=(,)
            ).start()
    
    def control_funcs(self, start): 
        #lst_control_funcs = []
        for slave in self.slaves:
            # if slave.name not in self.broken_slave_names:
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
    def __init__(self, name, idno, port_topics, timeout=0, **kwargs):
        self.name = name
        self.id = idno # id number of slave
        self.lst_readings = multiprocessing.Queue()
        self.time_readings = multiprocessing.Queue()
        if self.name in ['ADAM_TC', 'ADAM_TC_02', 'Scale', 'DFM', 'DFM_AOG']:
            self.size_lst_readings = {'short_lst_readings':params.manager.list(), 'long_lst_readings':params.manager.list()}
            self.size_time_readings = {'short_time_readings':params.manager.list(), 'long_time_readings':params.manager.list()}

        #self.readings = [] # for all data
        self.port_topics = port_topics
        self.kwargs = kwargs # dict of funcs
        self.timeout = timeout

    def read_rtu(self, *_fields, wait_len):
        self.r_wait_len = wait_len
        # _fields[0]:data_site
        # _fields[1]:value / data_len
        if len(_fields) == 2:
            data_struc = tohex_pad2(self.id) + '03' + _fields[0] + _fields[1]
            # print(data_struc)
            crc = Crc16Modbus.calchex(bytearray.fromhex(data_struc))
            self.r_rtu = data_struc + crc[-2:] + crc[:2]
        elif len(_fields) == 1:
            self.r_rtu = _fields[0]

    def write_rtu(self, *_fields):
        # _fields[0]:data_site
        # _fields[1]:value / data_len
        if len(_fields) == 2:
            _value = tohex_pad4(_fields[1])
            data_struc = tohex_pad2(self.id) + '06' + _fields[0] + _value
            crc = Crc16Modbus.calchex(bytearray.fromhex(data_struc))
            self.w_rtu = data_struc + crc[-2:] + crc[:2]
        elif len(_fields) == 1:
            if 'MFC' in self.name:
                self.w_rtu = f'\r{self.id}S{_fields[0]}\r\r'

    def PWM_instance(self, mode: str, frequency=1, duty=0):
        # Modbus.PIG.stop()
        if mode == 'software': # frequency up tp 1-150Hz, duty = 0-100%
            Modbus.PIG.set_mode(self.id, Modbus.pigpio.OUTPUT)
            # dutycycle:= 0-range (range defaults to 255).
            ## dutyrange set to 100
            Modbus.PIG.set_PWM_range(self.id, 10000) 
            # Each GPIO can be independently set to one of 18 different PWM frequencies.
            ## 8000  4000  2000 1600 1000  800  500  400  320
            ## 250   200   160  100   80   50   40   20   10
            Modbus.PIG.set_PWM_frequency(self.id, 10)
            #print(Modbus.PIG.get_PWM_frequency(self.id))
            Modbus.PIG.set_PWM_dutycycle(self.id, 0)
            #print(Modbus.PIG.get_PWM_dutycycle(self.id))
        elif mode == 'hardware': # frequency up tp 0-30kHz (or more), duty = 0-100%
            Modbus.PIG.set_mode(self.id, Modbus.pigpio.ALT5)
            Modbus.PIG.hardware_PWM(self.id, 0, 0)           
    def control_constructor(self):
        self.controller = PIDsim.PID(name=f'{self.name}_controller')
    
    def control_constructor_fixed(self, Kp, Ki, Kd, beta, kick, tstep, MVmax, MVmin, SP_range, SP_increment):
        self.controller = PIDsim.PID(name=f'{self.name}_controller')
        self.controller.update_paramater(Kp, Ki, Kd, beta, kick, tstep, MVmax, MVmin, SP_range, SP_increment)
#-------------------------RTU & Slave--------------------------------------
# ADAM_TC
# RTU func code 03, PV value site starts at '0000', data_len is 8 ('0008')
# ADAM_TC_slave = Slave(
#                     name = 'ADAM_TC',
#                     idno=ADAM_TC_id,
#                     port_topics=port_Topics(sub_topics=[
#                                             ],
#                                             pub_topics=[
#                                                 'BR', 'SR_front', 'SR_mid', 'SR_end', 
#                                                 'flue_gas', 'RAD_in', 'RAD_out', 'TC_7'
#                                                 'BN_rate', 'SR_rate',
#                                             ],
#                                             err_topics=[
#                                                 'ADAM_TC_collect_err', 'ADAM_TC_analyze_err',
#                                             ]
#                                             ),
#                     timeout = 0.02,
#                     comm_func=Modbus.Modbus_Comm,
#                     analyze_func=Modbus.ADAM_TC_analyze
#                     )
# ADAM_TC_slave.read_rtu('0000', '0008', wait_len=21)
# ADAM_TC_slave.w_wait_len = 8

# # GA slave
# GA_slave = Slave(
#                 name = 'GA',
#                 idno=GA_id,
#                 port_topics=port_Topics(sub_topics=[],
#                                         pub_topics=[
#                                             'GA_CO', 'GA_CO2', 'GA_CH4',
#                                             'GA_H2', 'GA_N2', 'GA_HEAT',
#                                             'H2','CO2','CO','MeOH','H2O'
#                                         ],
#                                         err_topics=[
#                                             'GA_collect_err', 'GA_analyze_err',
#                                         ]
#                                         ),
#                 timeout = 0.1,
#                 comm_func=Modbus.Modbus_Comm,
#                 analyze_func=Modbus.GA_data_analyze
#                 )
# GA_slave.read_rtu('11 01 60 8E', wait_len=31)

# Scale slave
# Scale_slave = Slave(
#                     name = 'Scale',
#                     idno=Scale_id,
#                     port_topics=port_Topics(sub_topics=[],
#                                             pub_topics=[
#                                                 '10_Scale', '60_Scale'
#                                             ],
#                                             err_topics=[
#                                                 'Scale_collect_err', 'Scale_analyze_err',
#                                             ]
#                                             ),
#                     timeout = 0.1,
#                     comm_func=Modbus.Scale_data_collect,
#                     analyze_func=Modbus.Scale_data_analyze
#                     )
# Scale_slave.read_rtu(wait_len=0)

# # TCHeader Rreading, RTU func code 03, PV value site at '008A', data_len is 1 ('0001')
# # TCHeader Writing, RTU func code 06, SV value site at '0000'
# # r_wait_len=7,
# # w_wait_len=8,
# Header_EVA_slave = Slave(
#                         name = 'Header_EVA',
#                         idno=Header_EVA_id,
#                         port_topics=port_Topics(
#                                 sub_topics=[
#                                 ],
#                                 pub_topics=[
#                                     'Header_EVA_PV', # Header EVA(Header_EVA_PV)
#                                 ],
#                                 err_topics=[
#                                     'Header_EVA_collect_err', 'Header_EVA_set_err', 'Header_EVA_analyze_err',
#                                 ]
#                                 ),
#                         timeout = 0.02,
#                         comm_func=Modbus.Modbus_Comm,
#                         analyze_func=Modbus.TCHeader_analyze
#                         )
# Header_EVA_slave.read_rtu('008A', '0001', wait_len=7)
# Header_EVA_slave.w_wait_len = 8

# Header_EVA_SET_slave = Slave(
#                         name = 'Header_EVA_SET',
#                         idno=Header_EVA_id,
#                         port_topics=port_Topics(
#                                 sub_topics=[
#                                     'Header_EVA_SV', 
#                                 ],
#                                 pub_topics=[
#                                     'Header_EVA_SET_PV'
#                                 ],
#                                 err_topics=[
#                                     'Header_EVA_SET_collect_err', 'Header_EVA_SET_set_err', 'Header_EVA_SET_analyze_err',
#                                 ]
#                                 ),
#                         timeout = 0.02,
#                         comm_func=Modbus.Modbus_Comm,
#                         analyze_func=Modbus.TCHeader_analyze
#                         )
# Header_EVA_SET_slave.read_rtu('0000', '0001', wait_len=7)
# Header_EVA_SET_slave.w_wait_len = 8

# Header_BR_slave = Slave(
#                         name='Header_BR',
#                         idno=Header_BR_id, 
#                         port_topics=port_Topics(
#                                 sub_topics=[
#                                 ],
#                                 pub_topics=[
#                                     'Header_BR_PV', # Header BR(Header_BR_PV)
#                                 ],
#                                 err_topics=[
#                                     'Header_BR_collect_err', 'Header_BR_set_err', 'Header_BR_analyze_err',
#                                 ]
#                                 ),
#                         timeout = 0.02,
#                         comm_func=Modbus.Modbus_Comm,
#                         analyze_func=Modbus.TCHeader_analyze
#                         )
# Header_BR_slave.read_rtu('008A', '0001', wait_len=7)
# Header_BR_slave.w_wait_len = 8

# Header_BR_SET_slave = Slave(
#                         name='Header_BR_SET',
#                         idno=Header_BR_id, 
#                         port_topics=port_Topics(
#                                 sub_topics=[
#                                     'Header_BR_SV', # Header BR(Header_BR_SV)
#                                 ],
#                                 pub_topics=[
#                                     'Header_BR_SET_PV' # Header BR(Header_BR_PV)
#                                 ],
#                                 err_topics=[
#                                     'Header_BR_SET_collect_err', 'Header_BR_SET_set_err', 'Header_BR_SET_analyze_err',
#                                 ]
#                                 ),
#                         timeout = 0.02,
#                         comm_func=Modbus.Modbus_Comm,
#                         analyze_func=Modbus.TCHeader_analyze
#                         )
# Header_BR_SET_slave.read_rtu('0000', '0001', wait_len=7)
# Header_BR_SET_slave.w_wait_len = 8

# ADAM_SET_slave, RTU func code 03, channel site at '0000-0003', data_len is 4 ('0004')
## ch00:+-10V, ch01:0-5V, ch02:0-5V, ch03:0-5V
# r_wait_len=13, # wait for 7 bytes == 7 Hex numbers
# w_wait_len=8,
ADAM_SET_slave = Slave(
                        name='ADAM_SET',
                        idno=ADAM_SET_id,
                        port_topics=port_Topics(
                                sub_topics=[
                                    'TOM_SET_SV', 'SOL_1_SET_SV', 'KNF_SET_SV',# 'SOL_2_SET_SV' # PCB(ADAM_SET_SV0), Pump(ADAM_SET_SV1), Air_MFC(ADAM_SET_SV2), H2_MFC(ADAM_SET_SV3)
                                ],
                                pub_topics=[
                                    'TOM_SET_PV', 'SOL_1_SET_PV', 'KNF_SET_PV',# 'SOL_2_SET_SV', # PCB(ADAM_SET_PV0), Pump(ADAM_SET_PV1), Air_MFC(ADAM_SET_PV2), H2_MFC(ADAM_SET_PV3)
                                ],
                                err_topics=[
                                    'ADAM_SET_collect_err', 'ADAM_SET_set_err', 'ADAM_SET_analyze_err',
                                ]
                                ),
                        timeout = 0.01,
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
                                    'SMC_0_PV', 'SMC_1_PV', 'ADAM_READ_PV2', 'ADAM_READ_PV3', 'ADAM_READ_PV4','ADAM_P_BR', 'ADAM_P_EVA', 'ADAM_P_ACC' # ADAM_READ_PV0 (SMC), ADAM_READ_PV1 (SMC), ADAM_READ_PV2, ADAM_READ_PV3, ADAM_READ_PV4(pump), ADAM_READ_PV5(Air_MFC), ADAM_READ_PV6(H2_MFC), ADAM_READ_PV7
                                ],
                                err_topics=[
                                    'ADAM_READ_collect_err', 'ADAM_READ_analyze_err',
                                ]
                                ),
                        timeout = 0.01,
                        comm_func=Modbus.Modbus_Comm,
                        analyze_func=Modbus.ADAM_READ_analyze
                        )
ADAM_READ_slave.read_rtu('0000', '0008', wait_len=21)

# DFMs' slaves
# DFM_slave = Slave(
#                 name='DFM',
#                 idno=DFM_id,
#                 port_topics=port_Topics(
#                                 sub_topics=['DFM_RichGas_1min','current','Convertion'],
#                                 pub_topics=[
#                                     '10_DFM_RichGas', '60_DFM_RichGas',
#                                 ],
#                                 err_topics=[
#                                     'DFM_collect_err', 'DFM_analyze_err', 
#                                 ]
#                                 ),
#                 timeout = 0.1,
#                 # comm_func=Modbus.VOID,
#                 analyze_func=Modbus.DFM_data_analyze
#                 )

# DFM_AOG_slave = Slave(
#                     name='DFM_AOG',
#                     idno=DFM_AOG_id, 
#                     port_topics=port_Topics(
#                                 sub_topics=['DFM_AOG_1min','Ratio'],
#                                 pub_topics=[
#                                     '10_DFM_AOG', '60_DFM_AOG',
#                                 ],
#                                 err_topics=[
#                                     'DFM_AOG_collect_err', 'DFM_AOG_analyze_err'
#                                 ]
#                                 ),
#                     timeout = 0.1,
#                     # comm_func=Modbus.VOID,
#                     analyze_func=Modbus.DFM_data_analyze
#                     )

# Air_MFC_slave = Slave(
#                     name='Air_MFC',
#                     idno=Air_MFC_id, 
#                     port_topics=port_Topics(
#                                 sub_topics=[
#                                     'Air_MFC_SET_SV',
#                                 ],
#                                 pub_topics=[
#                                     'Air_MFC_P', 'Air_MFC_T', 'Air_MFC_LPM', 'Air_MFC_SLPM', 'Air_MFC_SET_PV',
#                                 ],
#                                 err_topics=[
#                                     'Air_MFC_collect_err', 'Air_MFC_set_err', 'Air_MFC_analyze_err'
#                                 ]
#                                 ),
#                     comm_func=Modbus.MFC_Comm,
#                     analyze_func=Modbus.MFC_analyze,
#                     )
# Air_MFC_slave.read_rtu(f'\r{Air_MFC_id}\r\r', wait_len=49)
# Air_MFC_slave.w_wait_len = 49

# H2_MFC_slave = Slave(
#                     name='H2_MFC',
#                     idno=H2_MFC_id, 
#                     port_topics=port_Topics(
#                                 sub_topics=[
#                                     'H2_MFC_SET_SV',
#                                 ],
#                                 pub_topics=[
#                                     'H2_MFC_P', 'H2_MFC_T', 'H2_MFC_LPM', 'H2_MFC_SLPM', 'H2_MFC_SET_PV',
#                                 ],
#                                 err_topics=[
#                                     'H2_MFC_collect_err', 'H2_MFC_set_err', 'H2_MFC_analyze_err'
#                                 ]
#                                 ),
#                     comm_func=Modbus.MFC_Comm,
#                     analyze_func=Modbus.MFC_analyze,
#                     )
# H2_MFC_slave.read_rtu(f'\r{H2_MFC_id}\r\r', wait_len=49)
# H2_MFC_slave.w_wait_len = 49

Relay01_slave = Slave(
                    name='Relay01',
                    idno=channel_Relay01_IN1, 
                    port_topics=port_Topics(
                                sub_topics=[
                                    'Relay01_Set',
                                ],
                                pub_topics=[
                                ],
                                err_topics=[
                                    'Relay01_collect_err', 'Relay01_set_err', 'Relay01_analyze_err'
                                ]
                                ),
                    comm_func=Modbus.Relay_comm,
                    #analyze_func=Modbus.,
                    )
Relay02_slave = Slave(
                    name='Relay02',
                    idno=channel_Relay01_IN2, 
                    port_topics=port_Topics(
                                sub_topics=[
                                    'Relay02_Set',
                                ],
                                pub_topics=[
                                ],
                                err_topics=[
                                    'Relay02_collect_err', 'Relay02_set_err', 'Relay02_analyze_err'
                                ]
                                ),
                    comm_func=Modbus.Relay_comm,
                    #analyze_func=Modbus.,
                    )

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
                    timeout = 0.1,            
                    comm_func=Modbus.PWM_comm,
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
                    timeout = 0.1,            
                    comm_func=Modbus.PWM_comm,
                    #analyze_func=Modbus.,
                    )
PWM02_slave.PWM_instance('software')

WatchDog_slave = Slave(
                    name='WatchDog',
                    idno=WatchDog_id, 
                    port_topics=port_Topics(
                                sub_topics=[
                                    'Pump_SET_SV'
                                ],
                                pub_topics=[
                                ],
                                err_topics=[
                                ]
                                ),
                    timeout = 0.1,
                    # comm_func=Modbus.VOID,
                    # analyze_func=Modbus.VOID,
                    )

LambdaPID_slave = Slave(
                        name='LambdaPID',
                        idno=lambdapid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                'LambdaPID_Kp', 'LambdaPID_Ki', 'LambdaPID_Kd', 'LambdaPID_MVmin', 'LambdaPID_MVmax',
                                'LambdaPID_PV', 'LambdaPID_SP', 'LambdaPID_mode', 'LambdaPID_setting', 'LambdaPID_beta',
                                'LambdaPID_tstep', 'LambdaPID_kick', 'LambdaPID_woke',
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
#CV_01: Lambda value; MV: Air
## lambda_PV > lambda_SP => Air down => DirectAction=False
LambdaPID_slave.control_constructor()

CurrentPID_slave = Slave(
                        name='CurrentPID',
                        idno=currentpid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                'CurrentPID_Kp', 'CurrentPID_Ki', 'CurrentPID_Kd', 'CurrentPID_MVmin', 'CurrentPID_MVmax',
                                'CurrentPID_PV', 'CurrentPID_SP', 'CurrentPID_mode', 'CurrentPID_setting', 'CurrentPID_beta',
                                'CurrentPID_tstep', 'CurrentPID_kick', 'CurrentPID_woke',
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
# CV_02: SetCurrent; MV: RF_Pump flow rate
## current_PV > current_SP => RF_Pump down => DirectAction=False
CurrentPID_slave.control_constructor()

CatBedPID_slave = Slave(
                        name='CatBedPID',
                        idno=catbedpid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                'CatBedPID_Kp', 'CatBedPID_Ki', 'CatBedPID_Kd', 'CatBedPID_MVmin',  'CatBedPID_MVmax', 
                                'CatBedPID_PV', 'CatBedPID_SP', 'CatBedPID_mode', 'CatBedPID_setting', 'CatBedPID_beta',
                                 'CatBedPID_tstep', 'CatBedPID_kick', 'CatBedPID_woke',
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
# CV_03: CatBed TC; MV: Fuel to BR
## CatBed_PV > CatBed_SP => BR_Fuel down => DirectAction=False
CatBedPID_slave.control_constructor()

PCBPID_slave = Slave(
                        name='PCBPID',
                        idno=pcbpid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                'PCBPID_Kp', 'PCBPID_Ki', 'PCBPID_Kd', 'PCBPID_MVmin',  'PCBPID_MVmax', 
                                'PCBPID_PV', 'PCBPID_SP', 'PCBPID_mode', 'PCBPID_setting', 'PCBPID_beta',
                                'PCBPID_tstep', 'PCBPID_kick', 'PCBPID_woke',
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
# CV_04: PCB %; MV: ratio of AOG
## PCBP_PV > PCBP_SP => AOG down => DirectAction=False
PCBPID_slave.control_constructor()

PumpPID_slave = Slave(
                        name='PumpPID',
                        idno=pumppid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                'PumpPID_Kp', 'PumpPID_Ki', 'PumpPID_Kd', 'PumpPID_MVmin',  'PumpPID_MVmax', 
                                'PumpPID_PV', 'PumpPID_SP', 'PumpPID_mode', 'PumpPID_setting', 'PumpPID_beta',
                                'PumpPID_tstep', 'PumpPID_kick', 'PumpPID_woke',
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
                                'BurnerPID_MVmin', 'BurnerPID_MVmax', 'BurnerPID_PV', 'BurnerPID_SP', 'BurnerPID_mode', 'BurnerPID_setting', 'BurnerPID_woke', 
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
                        control_func=Modbus.control_fixed,
                        )
# CV_06: burner temperture; MV: PCB SP
## burner_PV > burner_SP => PCB down => DirectAction=False
# BurnerPID_slave.control_constructor()
BurnerPID_slave.control_constructor_fixed(Kp=0.0003, Ki=0.000003, Kd=0.001, beta=0.5, kick=4, tstep=5, MVmax=0.5, MVmin=0.15, SP_range=0, SP_increment=3)

EVAPID_slave = Slave(
                        name='EVAPID',
                        idno=evapid_id, 
                        port_topics=port_Topics(
                            sub_topics=[
                                # 'EVAPID_Kp', 'EVAPID_Ki', 'EVAPID_Kd', 
                                'EVAPID_MVmin',  'EVAPID_MVmax', 'EVAPID_PV', 'EVAPID_SP', 'EVAPID_mode', 'EVAPID_setting', 'EVAPID_woke',
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
                        control_func=Modbus.control_fixed,
                        )

EVAPID_slave.control_constructor_fixed(Kp=1, Ki=0.3, Kd=1, beta=1, kick=1, tstep=1, MVmax=100, MVmin=80, SP_range=0, SP_increment=3)

print('Slaves are all set')

#-----Port setting----------------------------------------------------------------
# Scale_port = device_port(Scale_slave,
#                         name='Scale_port',
#                         port=serial.Serial(port=Scale_port_path,
#                                             baudrate=9600, 
#                                             bytesize=8, 
#                                             stopbits=1, 
#                                             parity='N'),
#                         )

# RS232_port = device_port(GA_slave,
#                         name='RS232_port',
#                         port=serial.Serial(port=RS232_port_path,
#                                             baudrate=9600, 
#                                             bytesize=8, 
#                                             stopbits=1, 
#                                             parity='N'),
#                         )
# somehow the headers are affecting ADAMs
Setup_port = device_port(
                        # Header_BR_slave,
                        # Header_BR_SET_slave,
                        # Header_EVA_slave,
                        # Header_EVA_SET_slave,
                        # ADAM_TC_slave,
                        ADAM_READ_slave,
                        ADAM_SET_slave,
                        name='Setup_port',
                        port=serial.Serial(port=Setup_port_path,
                                            baudrate=115200, 
                                            bytesize=8, 
                                            stopbits=1, 
                                            parity='N'),
                        )

GPIO_port = device_port(
                        Relay01_slave,
                        Relay02_slave,
                        #DFM_slave,
                        #DFM_AOG_slave,
                        PWM01_slave,
                        PWM02_slave,
                        name='GPIO_port',
                        port='GPIO',
                        )

WatchDog_port = device_port(WatchDog_slave,
                        name='WatchDog_port',
                        port='WatchDog',
                        )

PID_port = device_port(
                    LambdaPID_slave,
                    CurrentPID_slave,
                    PumpPID_slave,
                    PCBPID_slave,
                    CatBedPID_slave,
                    name='PID_port',
                    port='PID',
                    )

lst_ports = [
            # MFC_port,
            # Scale_port, 
            # RS232_port, 
            Setup_port,
            GPIO_port,
            # WatchDog_port,
            # PID_port
            ]

NodeRed = params.manager.dict()

print('Ports are all set')