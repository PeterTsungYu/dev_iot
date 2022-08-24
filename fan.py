import pigpio
import time

PIG = pigpio.pi()

upper_temp = 50
lower_temp = 45
channel = 14
PIG.set_mode(channel, pigpio.OUTPUT)
PIG.set_pull_up_down(channel, pigpio.PUD_UP)
 
def get_temp():
    with open('/sys/class/thermal/thermal_zone0/temp') as fp:
        return int(fp.read()) / 1000
 
while True:
    temp = get_temp()
    if temp >= upper_temp:
        PIG.write(channel, 0)
    elif temp < lower_temp:
        PIG.write(channel, 1)
    time.sleep(10)
PIG.stop()