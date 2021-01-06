# %%
#!/usr/bin/env python

# %% 
import time
import serial
import RPi.GPIO as GPIO


# write from GPIO
## disable GPIO console login, and be as a serial port
TXRX = serial.Serial(    
    port='/dev/ttyUSB0', # check GPIO with $ dmesg | grep tty
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
counter=0

while True:
    print(counter)
    TXRX.write(b'yes')
    #TXRX.write(b'Write counter: %d \n'%(counter))
    #print(str.encode(f'{counter}'))
    time.sleep(1)
    counter += 1