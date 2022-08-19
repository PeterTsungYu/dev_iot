from RPi import GPIO
import time
 
upper_temp = 50
lower_temp = 45

GPIO.setmode(GPIO.GPIO.BCM)
# GPIO.setwarnings(False)
channel = 14
GPIO.setup(channel, GPIO.OUT, initial = GPIO.HIGH)
 
def get_temp():
    with open('/sys/class/thermal/thermal_zone0/temp') as fp:
        return int(fp.read()) / 1000
 
while True:
    temp = get_temp()
    if temp >= upper_temp:
        GPIO.output(channel, GPIO.LOW)
    elif temp < lower_temp:
        GPIO.output(channel, GPIO.HIGH)
    time.sleep(10)

GPIO.cleanup()