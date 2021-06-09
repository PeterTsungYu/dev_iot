#!/usr/bin/env python
#from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
# --------------------------------------------------------------------------- #
# configure the client logging
# --------------------------------------------------------------------------- #
import logging
FORMAT = ('%(asctime)-15s %(threadName)-15s '
          '%(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
logging.basicConfig(format=FORMAT)
log = logging.getLogger()
log.setLevel(logging.DEBUG)

UNIT = 0x06


def run_sync_client():
    #client = ModbusClient('localhost', port=5020)
    # from pymodbus.transaction import ModbusRtuFramer
    client = ModbusClient(
        method='rtu', 
        port='/dev/ttyS0', 
        timeout=1, 
        baudrate=115200,
        stopbits=1,
        bytesize=8,
        parity='N',
        retry_on_empty=True 
        )
    client.connect()

    log.debug("Write to a holding register and read back")
    rq = client.write_register(address=1, value=10, unit=UNIT)
    rr = client.read_holding_registers(1, 1, unit=UNIT)
    assert(not rq.isError())     # test that we are not an error
    assert(rr.registers[0] == 10)       # test the expected value

    log.debug("Write to multiple holding registers and read back")
    rq = client.write_registers(1, [10]*8, unit=UNIT)
    rr = client.read_holding_registers(1, 8, unit=UNIT)
    assert(not rq.isError())     # test that we are not an error
    assert(rr.registers == [10]*8)      # test the expected value

    arguments = {
        'read_address':    1,
        'read_count':      8,
        'write_address':   1,
        'write_registers': [20]*8,
    }
    log.debug("Read write registeres simulataneously")
    rq = client.readwrite_registers(unit=UNIT, **arguments)
    rr = client.read_holding_registers(1, 8, unit=UNIT)
    assert(not rq.isError())     # test that we are not an error
    assert(rq.registers == [20]*8)      # test the expected value
    assert(rr.registers == [20]*8)      # test the expected value

    client.close()


if __name__ == "__main__":
    run_sync_client()