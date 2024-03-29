#import threading
import multiprocessing
import signal

#-------------------------Global var--------------------------------------
db_comm_time      = 1
mqtt_comm_time    = 0.1
recur_try         = 1
exempt_try        = 30
exempt_threshold  = 10

# shared memory 
manager = multiprocessing.Manager()

#-----------------Interrupt events------------------------------
# Keyboard interrupt event to kill all the threads (Ctr + C)
kb_event = multiprocessing.Event()
def signal_handler(signum, frame):
    kb_event.clear()
    kb_event.set()
signal.signal(signal.SIGINT, signal_handler) # Keyboard interrupt to stop the program


def terminate(event): # ask user input to stop the program
    print(event.is_set())
    text = input("Type 'exit' to terminate the program...\n>")
    if text == 'exit':
        event.set()