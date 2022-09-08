import pigpio
import time
import threading
import signal

kb_event = threading.Event()
def signal_handler(signum, frame):
    if kb_event.is_set():
        kb_event.clear()
        kb_event.set()
    else:
        kb_event.set()
signal.signal(signal.SIGINT, signal_handler) # Keyboard interrupt to stop the program
PIG = pigpio.pi()

upper_temp = 50
lower_temp = 45
channel = 14
PIG.set_mode(channel, pigpio.OUTPUT)
PIG.set_pull_up_down(channel, pigpio.PUD_UP)
 
def get_temp():
    with open('/sys/class/thermal/thermal_zone0/temp') as fp:
        return int(fp.read()) / 1000
 
while not kb_event.is_set():
    temp = get_temp()
    if temp >= upper_temp:
        PIG.write(channel, 0)
    elif temp < lower_temp:
        PIG.write(channel, 1)
    time.sleep(10)
PIG.stop()