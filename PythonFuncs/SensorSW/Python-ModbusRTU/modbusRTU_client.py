from pymodbus.client.sync import ModbusSerialClient
import time

from pymodbus.version import version
from pymodbus.server.sync import StartTcpServer
from pymodbus.server.sync import StartTlsServer
from pymodbus.server.sync import StartUdpServer
from pymodbus.server.sync import StartSerialServer

from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSparseDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext

from pymodbus.transaction import ModbusRtuFramer, ModbusBinaryFramer

import logging
# Setup logging
FORMAT = ('%(asctime)-15s %(threadName)-15s '
          '%(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
logging.basicConfig(format=FORMAT)
log = logging.getLogger()
log.setLevel(logging.DEBUG)

UNIT = 0x2 #Slave number
number_of_holding_regs = 20

# Initializing Modbus Serial Client (RTU)
client = ModbusSerialClient(method = "rtu", port="COM7",stopbits = 1, bytesize = 8, parity = 'N', baudrate= 9600, timeout=1)

# Connect to client
def connect_rtu_client(client):
    '''Connecting to the Modbus RTU network. Make sure to use the right COM port and baudrate for the client.'''

    log.debug("Connecting to client")

    if client.connect() == True:
        log.debug("Connection established")
        return client

    elif client.connect() == False:
        log.debug("Connection to client failed. Slave %d not responding.", UNIT)

def read_holding_registers(client, address, count, unit):
    '''Reading Holding registers. Address=0 is the first register.'''
    log.debug("Reading Holding Register %d with count=%d on slave %d", address, count, UNIT)
    rr = client.read_holding_registers(address=address, count=count, unit=unit)
    log.debug("Response from slave %d %s", UNIT, rr.registers)

    return rr

def write_holding_registers(client, address, value, unit):
    '''Writing Holding registers. Address=0 is the first register.'''
    log.debug("Writing Holding Register %d the value=%d on slave %d", address, value, UNIT)
    wr = client.write_register(address=address, value=value, unit=unit)

    return wr

if __name__ == "__main__":

    client = connect_rtu_client(client)


    wr = write_holding_registers(client, 8, 69, UNIT)
    rr = read_holding_registers(client, 0, 9, unit=UNIT)

    assert(wr.function_code < 0x80)     # test that we are not an error
    assert(rr.registers[8] == 69)    # test the expected value
   
    print(wr)
    print(rr)
