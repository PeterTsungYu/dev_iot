#!/usr/bin/env python

import time
import serial

# read from USB
USB = serial.Serial(    
    port='/dev/ttyUSB0', # check USB port with $ dmesg | grep tty
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
counter=0

while True:
    x=USB.read(10)
    print(x)