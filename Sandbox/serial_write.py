#!/usr/bin/env python

# %% 
import time
import serial
import RPi.GPIO as GPIO


# write from
TXRX = serial.Serial(    
    port="/dev/ttyUSB0",
    baudrate = 19200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
counter=0

while True:
    print(counter)
    TXRX.write(b'03 03 00 00 00 01 85 E8')
    #TXRX.write(b'Write counter: %d \n'%(counter))
    #print(str.encode(f'{counter}'))
    time.sleep(1)
    counter += 1