from pymodbus.client.sync import ModbusSerialClient
import time

import logging
# Setup logging
# FORMAT = ('%(asctime)-15s %(threadName)-15s '
#           '%(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
# logging.basicConfig(format=FORMAT)
# log = logging.getLogger()
# log.setLevel(logging.DEBUG)

UNIT = 2 #Slave number

# Initializing Modbus Serial Client (RTU)
client = ModbusSerialClient(method = "rtu", port="COM4",stopbits = 1, bytesize = 8, parity = 'N', baudrate= 9600, timeout=1)
print(client.connect())

client.write_register(address=8, value=1, unit=UNIT)

rr = client.read_holding_registers(address=0, count=9, unit=UNIT)
print(rr)





