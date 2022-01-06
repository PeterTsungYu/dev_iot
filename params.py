import threading
import signal
import time

#-------------------------Global var--------------------------------------
time_out          = 0.1 # for collecting data
sample_time       = 1 # for analyzing data
sample_time_DFM   = 60

# count down events
ticker = threading.Event()
sample_ticker = threading.Event() # for sampling data
def sample_ticker():
    sample_ticker.set()
    time.sleep(sample_time)

sample_DFM_ticker = threading.Event() # for sampling data
def sample_DFM_ticker():
    sample_DFM_ticker.set()
    time.sleep(sample_time_DFM)

db_ticker = threading.Event() # for analyzing data


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