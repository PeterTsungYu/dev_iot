#python packages
import threading
import signal
import serial

#custom modules

#-------------------------Global var--------------------------------------
time_out = 0.1 # for collecting data
sample_time = 1 # for analyzing data
sample_time_Scale = 30
sample_time_DFM = 60

# count down events
ticker = threading.Event() # for analyzing data

# #barrier event for syncing all threads
#barrier_analyze = threading.Barrier(4) # Adam_data_analyze, Scale_data_analyze, GA_data_analyze, Main thread
#barrier_cast = threading.Barrier(4) # Adam_data_analyze, Scale_data_analyze, GA_data_analyze, Main thread
#barrier_kill = threading.Barrier(9) # all threads

#-----------------Serial port setting------------------------------
# (Optional) Set the USB devices to have a default name
#_port_path = '/dev/ttyUSB'
RS485_port_path = '/dev/ttyUSB0'
Scale_port_path = '/dev/ttyUSB1'

## device ID
TCHeader_0_id = '00'
TCHeader_1_id = '01'

#-----------------Serial port instances------------------------------
## RS485
### set the baudrate of TCHeader_1 to 19200 
### set the baudrate of Pump to 19200
RS485_port = serial.Serial(
    port=RS485_port_path,
    baudrate=115200, 
    bytesize=8, 
    stopbits=1, 
    parity='N'
    )

# Scale USB
Scale_port = serial.Serial(
    port=Scale_port_path,
    baudrate=9600, 
    bytesize=8, 
    stopbits=1, 
    parity='N'
    )

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




