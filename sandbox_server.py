#!/usr/bin/env python
import threading
import time
from pymodbus.server.sync import StartTcpServer
from pymodbus.server.sync import StartTlsServer
from pymodbus.server.sync import StartUdpServer
from pymodbus.server.sync import StartSerialServer

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

def run_server():
    # ----------------------------------------------------------------------- #
    # initialize the server information
    # ----------------------------------------------------------------------- #
    # If you don't set this or any fields, they are defaulted to empty strings.
    # ----------------------------------------------------------------------- #
    identity = ModbusDeviceIdentification()
    identity.VendorName = 'Pymodbus'
    identity.ProductCode = 'PM'
    identity.VendorUrl = 'http://github.com/riptideio/pymodbus/'
    identity.ProductName = 'Pymodbus Server'
    identity.ModelName = 'Pymodbus Server'
    identity.MajorMinorRevision = '2.3.0'

    # ----------------------------------------------------------------------- #
    # run the server you want
    # ----------------------------------------------------------------------- #
    # RTU:
    StartSerialServer(
        context, 
        framer=ModbusRtuFramer, 
        identity=identity,
        port='/dev/ttyUSB0', 
        timeout=1, 
        baudrate=115200,
        stopbits=1,
        bytesize=8,
        parity='N', 
        )


if __name__ == "__main__":
    store = ModbusSlaveContext(
        di=ModbusSequentialDataBlock(0, [1]*100),
        co=ModbusSequentialDataBlock(0, [1]*100),
        hr=ModbusSequentialDataBlock(0, [1]*100),
        ir=ModbusSequentialDataBlock(0, [1]*100))
    context = ModbusServerContext(slaves=store, single=True)

    t = threading.Thread(target=run_server)
    t.start()

    while True:
        values=context[0].getValues(fx=3, address=0, count=5)
        print(values)
        values = [v + 1 for v in values]
        context[0].setValues(fx=3, address=0, values=values)
        time.sleep(1)