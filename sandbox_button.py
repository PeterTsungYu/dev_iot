import time
import RPi.GPIO as GPIO
import threading
 
BUTTON_PIN = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
presser = threading.Event()

def button_press(BUTTON_PIN):
    print(GPIO.input(BUTTON_PIN), ' here')
    if GPIO.input(BUTTON_PIN) == GPIO.LOW:
        presser.set()
    else:
        presser.clear()
GPIO.add_event_detect(BUTTON_PIN, GPIO.BOTH, callback=button_press, bouncetime=200)

presser.wait()
 
try:
    print('按下 Ctrl-C 可停止程式')
    while True:
        print(GPIO.input(BUTTON_PIN))
        time.sleep(2)
except KeyboardInterrupt:
    print('關閉程式')
finally:
    GPIO.cleanup()