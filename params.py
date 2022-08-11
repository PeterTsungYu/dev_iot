#import threading
import multiprocessing
import signal

#-------------------------Global var--------------------------------------
time_out          = 0.1 # for collecting data
sample_time       = 0.5 # for analyzing data
recur_try         = 2
exempt_try        = 3000
exempt_threshold  = 10

# count down events
sample_ticker = multiprocessing.Event()

# shared memory 
manager = multiprocessing.Manager()

#-----------------Interrupt events------------------------------
# Keyboard interrupt event to kill all the threads (Ctr + C)
kb_event = multiprocessing.Event()
def signal_handler(signum, frame):
    kb_event.set()
signal.signal(signal.SIGINT, signal_handler) # Keyboard interrupt to stop the program


def terminate(event): # ask user input to stop the program
    print(event.is_set())
    text = input("Type 'exit' to terminate the program...\n>")
    if text == 'exit':
        event.set()