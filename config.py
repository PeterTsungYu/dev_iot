#python packages
import threading
import signal
import serial
import RPi.GPIO as GPIO

#custom modules

#-------------------------Global var--------------------------------------
time_out = 1 # for collecting data
sample_time = 2 # for analyzing data
sample_time_Scale = 5
sample_time_DFM = 60

# count down events
ticker = threading.Event() # for analyzing data

# #barrier event for syncing all threads
#barrier_analyze = threading.Barrier(4) # ADAM_TC_analyze, Scale_data_analyze, GA_data_analyze, Main thread
#barrier_cast = threading.Barrier(4) # ADAM_TC_analyze, Scale_data_analyze, GA_data_analyze, Main thread
#barrier_kill = threading.Barrier(9) # all threads

#-----------------Serial port setting------------------------------
# (Optional) Set the USB devices to have a default name
#_port_path = '/dev/ttyUSB'
RS485_port_path = '/dev/ttyUSB_RS485' # for monitoring (TC from ADAM)
Scale_port_path = '/dev/ttyUSB_Scale' # for monitoring Scale
RS232_port_path = '/dev/ttyUSB_RS232' # for monitoring GA
Setup_port_path = '/dev/ttyUSB_PC' # for controling (ADAM, TCHeader)

## device ID
TCHeader_1_id = '01' # ReformerTP EVA_Header @ Setup_port_path
TCHeader_2_id = '02' # ReformerTP BR_Header @ Setup_port_path
ADAM_SET_id = '03' # ReformerTP ADAM_4024 for setting @ Setup_port_path
ADAM_READ_id = '04' # ReformerTP ADAM_4017+ for monitoring via oltage and current @ Setup_port_path
ADAM_TC_id = '03' # ReformerTP ADAM_4018+ for monitoring temp @ RS485_port_path
GA_id = '11' # ReformerTP GA for monitoring gas conc. @ RS232_port_path
# scale has no id

#-----------------Serial port instances------------------------------
## RS485
### set the baudrate to 19200 
RS485_port = serial.Serial(
    port=RS485_port_path,
    baudrate=19200, 
    bytesize=8, 
    stopbits=1, 
    parity='N'
    )

## Scale USB
Scale_port = serial.Serial(
    port=Scale_port_path,
    baudrate=9600,
    bytesize=8, 
    stopbits=1, 
    parity='N'
    )

## RS232
### set the baudrate of GA & MFC to 9600
RS232_port = serial.Serial(
    port=RS232_port_path,
    baudrate=9600, 
    bytesize=8, 
    stopbits=1, 
    parity='N'
    )

## Setup port 
Setup_port = serial.Serial(
    port=Setup_port_path,
    baudrate=115200, 
    bytesize=8, 
    stopbits=1, 
    parity='N'
    )

#-----GPIO port setting----------------------------------------------------------------
## DFM
# read High as 3.3V
channel_DFM = 18
channel_DFM_AOG = 23
GPIO.setmode(GPIO.BCM)

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




