import serial
import time

'''
receiver = serial.Serial(     
     port='/dev/ttyUSB0',        
     baudrate = 115200,
     parity=serial.PARITY_NONE,
     stopbits=serial.STOPBITS_ONE,
     bytesize=serial.EIGHTBITS,
     timeout=1
     )

while True:
      x = receiver.readline()
      print x
'''

#!/usr/bin/env python
from pymodbus.server.sync import ModbusSerialServer
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer
import threading
# --------------------------------------------------------------------------- #
# configure the service logging
# --------------------------------------------------------------------------- #
import logging
FORMAT = ('%(asctime)-15s %(threadName)-15s'
          ' %(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
logging.basicConfig(format=FORMAT)
log = logging.getLogger()
log.setLevel(logging.DEBUG)

def gen_and_run_server():
    register_block = ModbusSequentialDataBlock(0x00, [0]*0xff)
    #print(register_block.__str__())

    store = ModbusSlaveContext(
        #di=ModbusSequentialDataBlock(0, [1]*100),
        #co=ModbusSequentialDataBlock(0, [2]*100),
        hr=register_block, # holding register block, for func = 3, 6, 16
        #ir=ModbusSequentialDataBlock(0, [4]*100),
        zero_mode=True
        )
    #print(store.getValues(fx=3, address=0x00, count=5))

    context = ModbusServerContext(
        slaves={slave_id:store,}, # collection of slaves; here only slave 6
        single=False
        )
    #print(context)    
    #print(f"This is the slave 6: {context[slave_id]}") # index as a dict; otherwise, as a list

    # RTU:
    RTU_server = ModbusSerialServer(
        context, 
        framer=ModbusRtuFramer,  
        port='/dev/ttyS0', 
        timeout=1, 
        baudrate=115200,
        stopbits=1,
        bytesize=8,
        parity='N', 
        )

    return RTU_server


def updating_writer(context):
    #print(context[slave_id])
    log.debug("updating the context")
    values = context[slave_id].getValues(fx=fx_code, address=address, count=5)
    values = [v + 1 for v in values]
    log.debug("new values: " + str(values))
    context[slave_id].setValues(fx=fx_code, address=address, values=values)


class update_and_timeout(threading.Thread):
    def __init__(self, context):
        threading.Thread.__init__(self)
        self._timeout = threading.Event()
        self.context = context
        print(self.context)

    def run(self):
        while not self._timeout.wait(1): # every beginning of iteration, timeout for 1s
            updating_writer(self.context)

    def join(self, timeout=None):
        """ Stop the thread. """
        self._timeout.set(  )
        threading.Thread.join(self, timeout)


if __name__ == "__main__":
    fx_code = 3
    slave_id = 0x06
    address = 0x10
    try:
        Rpi = gen_and_run_server()
        print("Server is online")

        update = update_and_timeout(Rpi.context)
        update.start()
        time.sleep(600.0)
        update.join()
        '''
        while True:
            run_updating_server = threading.Timer(interval=1, function=updating_writer, args=Rpi.context)
            run_updating_server.start()
            run_updating_server.join()
        '''

    except Exception as ex:
        print ("open serial port error " + str(ex))
        Rpi.server_close()
        print("Server is offline")


print(Rpi.context[0x06].getValues(fx=fx_code, address=address, count=5))
    