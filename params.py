#import threading
import multiprocessing
import signal

#-------------------------Global var--------------------------------------
time_out          = 0.1 # for collecting data
sample_time       = 1 # for analyzing data
sample_time_DFM   = 1

# count down events
time_out_ticker = multiprocessing.Event()
sample_ticker = multiprocessing.Event()
main_ticker = multiprocessing.Event()

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