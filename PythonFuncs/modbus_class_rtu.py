from pymodbus.client.sync import ModbusSerialClient

class RTU_Client():
    def __init__(self, port: str):
        '''Port argument must be string -> "COM7"'''
        self.port = port
        self.connected = False

    def connect_to_rtu_client(self):
        self.client = ModbusSerialClient(method = "rtu", port=self.port,
        stopbits = 1, bytesize = 8, parity = 'N', baudrate= 9600, timeout=1)

        connected = self.client.connect()

        return self, connected


    def read_holding_registers(self, address: int, count: int, unit: int) -> int:
        '''Reading Holding registers. Address=0 is the first register.'''

        rr = self.client.read_holding_registers(address=address, count=count, unit=unit)

        return rr

    def write_holding_registers(self, address: int, value: int, unit: int) -> None:
        '''Writing Holding registers. Address=0 is the first register.'''

        wr = self.client.write_register(address=address, value=value, unit=unit)

        return wr

    def disconnect_from_RTU_client(self):
        self.client.close()
        print('Disconnected from client')

if __name__ == "__main__":

    rtu = RTU_Client("COM7")
    print(rtu.connect_to_rtu_client())
   
    UNIT = 2

    wr = rtu.write_holding_registers(8, 69, UNIT)
    print(wr)
    rr = rtu.read_holding_registers(0, 9, UNIT)
    print(rr.registers)

