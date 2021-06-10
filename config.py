import threading
import signal

#-------------------------Global var--------------------------------------
time_out = 1 # for collecting data
sample_time = 2 # for analyzing data
sample_time_DFM = 60

# count down events
ticker = threading.Event() # for analyzing data

# #barrier event for syncing all threads
#barrier_analyze = threading.Barrier(4) # Adam_data_analyze, Scale_data_analyze, GA_data_analyze, Main thread
#barrier_cast = threading.Barrier(4) # Adam_data_analyze, Scale_data_analyze, GA_data_analyze, Main thread
#barrier_kill = threading.Barrier(9) # all threads


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


#-------------------------MQTT--------------------------------------
topic_ADAM_TC = [
    "/rpi/Reformer_TC_07", "/rpi/Reformer_TC_08", "/rpi/Reformer_TC_09", "/rpi/Reformer_TC_10",
    "/rpi/Reformer_TC_11", "/rpi/Reformer_TC_12", "/rpi/Reformer_TC_13", "/rpi/Reformer_TC_14"]

