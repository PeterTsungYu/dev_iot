import serial
import time

#!/usr/bin/env python
from pymodbus.server.sync import StartSerialServer, ModbusSingleRequestHandler
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

def serverDB_gen(slave_id=0x00):
    register_block = ModbusSequentialDataBlock(0x00, [0x00]*0x22) # each address can hold from a range 0x00 to 0xffff
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
    print("Succeed to generate a server context")
    return context


def run_server(context, port, timeout=1, baudrate=115200, stopbits=1, bytesize=8, parity='N'):
    StartSerialServer(
        context, 
        framer=ModbusRtuFramer,  
        port=port, 
        timeout=timeout, 
        baudrate=baudrate,
        stopbits=stopbits,
        bytesize=bytesize,
        parity=parity, 
        )
    print("Server is online")


def updating_writer(context):
    #print(context[slave_id])
    log.debug("updating the context")
    values = context[slave_id].getValues(fx=fx_code, address=address, count=5)
    print(values)
    values = [v + 1 for v in values]
    log.debug("new values: " + str(values))
    context[slave_id].setValues(fx=fx_code, address=address, values=values)


class update_and_timeout(threading.Thread):
    def __init__(self, context):
        threading.Thread.__init__(self)
        self._timeout = threading.Event()
        self.context = context
        #print(self.context)

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
    server_DB = serverDB_gen(slave_id=slave_id)
    server_thread = threading.Thread(
        target=run_server, 
        args=(server_DB, '/dev/ttyUSB0', 1, 115200, 1, 8, 'N')
        )
    update = update_and_timeout(server_DB)
    try:
        server_thread.start()
        time.sleep(1)
        update.start()
        time.sleep(600.0)
        update.join()
    except Exception as ex:
        print ("Server error: " + str(ex))
        #Rpi.server_close()
        print("Server is offline")
    print(server_DB[0x06].getValues(fx=fx_code, address=address, count=5))   