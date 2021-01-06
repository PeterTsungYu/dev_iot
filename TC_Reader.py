import serial, time

# Initiate an serial instance by call the class: Serial() 
ser = serial.Serial()

# set the USB port name
ser.port = "/dev/ttyUSB0"
  
#115200,N,8,1
# 9600 as default, baudrate (bits per second)
# 8 as default, bytesize
# N as default, parity_none
# 1 as default, stop bits
ser.baudrate = 115200
ser.timeout = 0.5          #non-block read 0.5s
ser.writeTimeout = 0.5     #timeout for write 0.5s
ser.bytesize = serial.EIGHTBITS #number of bits per bytes
ser.parity = serial.PARITY_NONE #set parity check
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
ser.xonxoff = False    #disable software flow control. False as default
ser.rtscts = False     #disable hardware (RTS/CTS) flow control. False as default
ser.dsrdtr = False     #disable hardware (DSR/DTR) flow control. False as default
  
try: 
    # open the port
    ser.open()
except Exception as ex:
    print ("open serial port error " + str(ex))
    exit()
  
if ser.isOpen():
  
    try:
        ser.reset_input_buffer() #flush input buffer
        ser.reset_output_buffer() #flush output buffer
  
        #write 8 byte data
        ser.write([78, 78, 78, 78, 78, 78, 78, 78])
        print("write 8 byte data: 78, 78, 78, 78, 78, 78, 78, 78")
  
        time.sleep(0.5)  #wait 0.5s
  
        #read 8 byte data
        response = ser.read(8)
        print("read 8 byte data:")
        print(response)
  
        ser.close()
    except Exception as e1:
        print ("communicating error " + str(e1))
  
else:
    print ("open serial port error")