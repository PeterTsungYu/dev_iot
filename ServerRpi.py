import serial
import time

receiver = serial.Serial(     
     port='/dev/ttyUSB0',        
     baudrate = 115200,
     parity=serial.PARITY_NONE,
     stopbits=serial.STOPBITS_ONE,
     bytesize=serial.EIGHTBITS,
     timeout=1
     )

'''
TC - 8 * 2 = 16 (USB)
DFM - 1 * 2 = 2 (GPIO)
GA - 6 * 2 = 12 (USB)
MFC - 1 * 2 = 2 (GPIO ??)
Scale - 1 * 2 = 2 (USB)
Total = 34 (0x22)
RTU: 06 03 00 00 00 22 C4 64
each data entry would have (0x00-0xffff, which is 0-65535)
'''

request = '060300000022C464'
while True:
    if str(receiver.read(16).hex()) == request:
    time.sleep(1)