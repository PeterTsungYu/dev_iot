import pigpio
import time
import signal

GPIO = pigpio.pi()
channel = 14
# Set the GPIO-Mode
GPIO.set_mode(channel, pigpio.OUTPUT)

def signal_handler(signum, frame):
    # Pull down the GPIO-Pin and cleanup with stop()
    GPIO.write(channel, 0)
    GPIO.stop()
signal.signal(signal.SIGTERM, signal_handler) # Stop systemctl to stop the program
 
upper_temp = 50
lower_temp = 45
 
def get_temp():
    with open('/sys/class/thermal/thermal_zone0/temp') as fp:
        return int(fp.read()) / 1000
 
while True:
    temp = get_temp()
    if temp >= upper_temp:
        GPIO.write(channel, 0)
    elif temp < lower_temp:
        GPIO.write(channel, 1)
    time.sleep(10)
