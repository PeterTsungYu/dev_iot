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
from pymodbus.server.sync import StartSerialServer, ModbusSerialServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSparseDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusBinaryFramer
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
    # ----------------------------------------------------------------------- #
    # initialize your data store
    # ----------------------------------------------------------------------- #
    # The datastores only respond to the addresses that they are initialized to
    # Therefore, if you initialize a DataBlock to addresses of 0x00 to 0xFF, a
    # request to 0x100 will respond with an invalid address exception. This is
    # because many devices exhibit this kind of behavior (but not all)::
    #
    #     block = ModbusSequentialDataBlock(0x00, [0]*0xff)
    #
    # Continuing, you can choose to use a sequential or a sparse DataBlock in
    # your data context.  The difference is that the sequential has no gaps in
    # the data while the sparse can. Once again, there are devices that exhibit
    # both forms of behavior::
    #
    #     block = ModbusSparseDataBlock({0x00: 0, 0x05: 1})
    #     block = ModbusSequentialDataBlock(0x00, [0]*5)
    #
    # Alternately, you can use the factory methods to initialize the DataBlocks
    # or simply do not pass them to have them initialized to 0x00 on the full
    # address range::
    #
    #     store = ModbusSlaveContext(di = ModbusSequentialDataBlock.create())
    #     store = ModbusSlaveContext()
    #
    # Finally, you are allowed to use the same DataBlock reference for every
    # table or you may use a separate DataBlock for each table.
    # This depends if you would like functions to be able to access and modify
    # the same data or not::
    #
    #     block = ModbusSequentialDataBlock(0x00, [0]*0xff)
    #     store = ModbusSlaveContext(di=block, co=block, hr=block, ir=block)
    #
    # The server then makes use of a server context that allows the server to
    # respond with different slave contexts for different unit ids. By default
    # it will return the same context for every unit id supplied (broadcast
    # mode).
    # However, this can be overloaded by setting the single flag to False and
    # then supplying a dictionary of unit id to context mapping::
    #
    #     slaves  = {
    #         0x01: ModbusSlaveContext(...),
    #         0x02: ModbusSlaveContext(...),
    #         0x03: ModbusSlaveContext(...),
    #     }
    #     context = ModbusServerContext(slaves=slaves, single=False)
    #
    # The slave context can also be initialized in zero_mode which means that a
    # request to address(0-7) will map to the address (0-7). The default is
    # False which is based on section 4.4 of the specification, so address(0-7)
    # will map to (1-8)::
    #
    #     store = ModbusSlaveContext(..., zero_mode=True)
    # ----------------------------------------------------------------------- #
    register_block = ModbusSequentialDataBlock(0x00, [0]*0xff)
    store = ModbusSlaveContext(
        #di=ModbusSequentialDataBlock(0, [1]*100),
        #co=ModbusSequentialDataBlock(0, [2]*100),
        hr=register_block, # holding register block, for func = 3, 6, 16
        #ir=ModbusSequentialDataBlock(0, [4]*100),
        zero_mode=True)
    print(store.getValues(3, 0x00, count=5))

    context = ModbusServerContext(
        slaves={0x06:store,}, 
        single=False
        )
    

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

if __name__ == "__main__":
    try:
        Rpi = gen_and_run_server()
        print(Rpi._connect())
        print("Server is online")
        while True:
            continue
    except:
        Rpi.server_close()
        print("Server is offline")
    