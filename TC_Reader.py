import serial, time


#-----------------Slaves(RPi) setting------------------------------
idno = {
    'T_b':'1', #Body Temp 
    'T_out':'2' #Output Temp
    }
#------------------------------------------------------------------

#-----------------Master(RPi) setting------------------------------
# Initiate an serial instance by call the class: Serial() 
ser = serial.Serial()

# set the USB port name
ser.port = "/dev/ttyUSB0"

# 9600 bps
ser.baudrate = 19200
# O_81
ser.bytesize = serial.EIGHTBITS # 8 bits per bytes
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
ser.parity = serial.PARITY_EVEN #set parity check
ser.timeout = 0.5          #non-block read 0.5s
ser.writeTimeout = 0.5     #timeout for write 0.5s
#ser.xonxoff = False    #disable software flow control. False as default
#ser.rtscts = False     #disable hardware (RTS/CTS) flow control. False as default
#ser.dsrdtr = False     #disable hardware (DSR/DTR) flow control. False as default
#------------------------------------------------------------------

#-----------------ModBus RTU------------------------------
# master read from slaves
##for slave 1: 01H 03H 008AH 0001H A5E0H
##for slave 2: 02H 03H 008AH 0001H A5E0H
slave_1 = '01 03 00 8A 00 01 A5 E0'
slave_2 = '02 03 00 9A 00 01 A5 E0'
#slave_1 = '01 04 00 01 00 01 60 0A'
#slave_2 = '02 04 00 01 00 03 E1 F8'
#------------------------------------------------------------------

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
  
        for slave in [slave_1, slave_2]: 
            #write 8 byte data
            ser.write(bytes.fromhex(slave)) #hex to binary(byte) 
            print("write to slave")
    
            time.sleep(1)  #wait 0.5s
            
            #read 8 byte data
            len_return_data = ser.inWaiting()  # 獲取緩沖數據（接收數據）長度
            #print(len_return_data)
            if len_return_data:
                return_data = ser.read(len_return_data)  # 讀取緩沖數據
            # bytes(2進制)轉換為hex(16進制)，應注意Python3.7與Python2.7此處轉換的不同，並轉為字符串后截取所需數據字段，再轉為10進制
                str_return_data = str(return_data.hex())
                print(str_return_data)
                feedback_data = int(str_return_data[-8:-4], 16)
                print(feedback_data)
  
        ser.close()
        
    except Exception as e1:
        print ("communicating error " + str(e1))
  
else:
    print ("open serial port error")