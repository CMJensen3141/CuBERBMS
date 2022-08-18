#import pymodbus
from pymodbus.client.sync import ModbusTcpClient as ModbusClient


class Client():
    def __init__(self, ip_address='192.168.1.17'):
        self.ip_address = ip_address
        self.connected = False
        
    def connect_to_client(self):
        """
        Connects to the client with the specified ip address and port number.
        Default values are:
        :ip: = "192.168.1.17"
        :port_number: = 502
        :retries: = 3
        """
        # client = ModbusClient(ip=ip, retries=retries, port=port_number)
        self.client = ModbusClient(host=self.ip_address, port=502)
        connected = self.client.connect()
        return self, connected
        

        # if client.connect():
        #     return client
        # elif not client.connect():
        #     raise Exception('Connection to the client could not be established')


    def start_transmission(self, ac_power: int):

        """
    Starts transmission of power. battery current and power needs to be assigned. Sets all values that needs to be set and turns on the AC and DC module. 
        1: active_power sets the active power the system must consume or produce. Resolution 1.0 [VA]
        """
        if self.connected:    
            # sets value for power distribution of AC-module
            self.client.write_register(address=4199, value=ac_power, unit=0)
            #Set power factor to 1 (100%)
            #self.client.write_register(address=4217, value=100, unit=0) 
            # Specifies the slave that will be addressed (0 = broadcast / same values for all slaves)
            self.client.write_register(address=4007, value=1, unit=0)
            # Specifies the subslave that will be addressed (0 = broadcast / same values for all slaves)
            self.client.write_register(address=4010, value=1, unit=0)
    
            # Power stage configuration: 1 = power stage on // 0 = power stage off
            self.client.write_coil(address=4000, value=1, unit=0)  # power_on = 1
            
    def stop_transmission(self) -> None:
        """
        Stops transmission of power. Both AC and DC-module shut down.
        """
        # Power stage configuration: 1 = power stage on // 0 = power stage off
        if self.connected:
            self.client.write_coil(address=4000, value=0, unit=0)
            print('Power is Off')
        #client.close()
    def disconnect_from_client(self):
        if self.connected:
            self.client.close()
            print('Disconnected from client')

    def charge_battery(self):
        """
        Sets convention that ensures that the battery will charge.
        """
        if self.connected:
            # Read reference frame convention (0=producer reference frame; 1=consumer reference frame)
            direction = self.client.read_holding_registers(address=4006,count=1,unit=0).registers[0]
            if direction == 0:
                # If producer reference frame set power factor to -1    
                self.client.write_register(address=4217, value=65535-100, unit=0) #power factor at 1.
            elif direction == 1:
                # If consumer reference frame set power factor to 1
                self.client.write_register(address=4217, value=100, unit=0) #power factor at 1.

    def discharge_battery(self):
        """
        Sets convention that ensures that the battery will discharge.
        """
        # Read reference frame convention (0=producer reference frame; 1=consumer reference frame)
        if self.connected:
            direction = self.client.read_holding_registers(address=4006,count=1,unit=0).registers[0]
            if direction == 1:
                # if consumer set power factor to -1
                self.client.write_register(address=4217, value=65535-100, unit=0) #power factor at 1.
            elif direction == 0:
                # if propducer set power factor to 1
                self.client.write_register(address=4217, value=100, unit=0) #power factor at 1.

    def set_constant_power(self, power_setpoint: int):
        if self.connected:
        # write Max. power per DC phase
            self.client.write_register(address=4121, value=power_setpoint, unit=0) 
    
            # Read battery voltage
            voltage = self.client.read_input_registers(address=5100, count=1, unit=101).registers[0]
            voltage_battery = float(voltage/ 10)
    
            current_setpoint = (power_setpoint/voltage_battery)+2
            if current_setpoint > 110:
                current_setpoint = 110
                print('Current setpoint limited to 15 A')
            current_setpoint = int(current_setpoint*10)
    
            # write Max. battery charging current
            self.client.write_register(address=4106, value=current_setpoint, unit=0)
    
            # write Max. battery discharging current
            self.client.write_register(address=4109, value=current_setpoint, unit=0)

    def set_constant_current(self, setpoint_current: float):
        if self.connected:
            if setpoint_current > 110:
                setpoint_current = 110
            current_setpoint = int(setpoint_current*10)
            # write Max. battery charging current
            self.client.write_register(address=4106, value=current_setpoint, unit=0)
            # write Max. battery discharging current
            self.client.write_register(address=4109, value=current_setpoint, unit=0)
    
            # Read battery voltage
            voltage = self.client.read_input_registers(address=5100, count=1, unit=101).registers[0]
            voltage_battery = float(voltage/ 10)
    
            power_setpoint = int((setpoint_current*voltage_battery)+300)
            # write Max. power per DC phase
            self.client.write_register(address=4121, value=power_setpoint, unit=0) 
         

        

    def set_AC_power(self, ac_power: int):
        """
        Sets Power set value for the AC module.
        1: ac_power: Power value, Resolution 1.0VA
        """
        if self.connected:
            # Write power nominal value AC
            self.client.write_register(address=4199, value=ac_power, unit=0) # sets value for power distribution of AC-module


    def set_battery_power(self, battery_power: int):
        """
        sets maximum permitted DC power for the DC1008
        1: battery_power sets maximum battery power. Resolution 1.0 [W]
        """
        if self.connected:
            # write Max. power per DC phase
            self.client.write_register(address=4121, value=battery_power, unit=0) 

    def set_charging_current(self, charging_current: float):
        """
        Set limits for battery when charging
        1: sets the maximum charging current for battery. Resolution 0.1 [A]
        """
        if self.connected:
            charge_current = int(charging_current*10)
            
            # Write Max. battery charging current
            self.client.write_register(address=4106, value=charge_current, unit=0)


    def set_discharging_current(self, discharging_current: float):
        """
        Set limits for battery when discharging
        1: sets the maximum discharging current for battery. Resolution 0.1 [A]
        """
        if self.connected:
            discharge_current = int(discharging_current*10)
    
            # Max. battery discharging current
            self.client.write_register(address=4109, value=discharge_current, unit=0)

    def set_min_battery_voltage(self, min_bat_voltage: float):
        """
        Set limits for battery when discharging
        1: sets the maximum discharging current for battery. Resolution 0.1 [A]
        """
        if self.connected:
            minimum_bat_voltage = int(min_bat_voltage*100)
            
            # write Min. battery voltage 
            self.client.write_register(address=4101, value=minimum_bat_voltage, unit=0)
    
    def set_max_battery_voltage(self, max_bat_voltage: float):
        """
        Set limits for battery when discharging
        1: sets the maximum discharging current for battery. Resolution 0.1 [A]
        """
        if self.connected:
            maximum_bat_voltage = int(max_bat_voltage*100)
            
            #Write Max. battery voltage
            self.client.write_register(address=4100, value=maximum_bat_voltage, unit=0)   

    def set_power_factor(self, power_factor: float):
        """
        Set limits for battery when discharging
        1: sets the maximum discharging current for battery. Resolution 0.1 [A]
        """
        if self.connected:
            pf= int(power_factor*100)
            
            # Write nominal value cos phi for L1-L3
            self.client.write_register(address=4217, value=pf, unit=0)   
    


    def get_battery_current(self) -> float:
        """
        Reads actual battery current.
        Return value is Battery current. Resolution is 1.0 [A]
        """
        if self.connected:
        # Read battery current
            current = self.client.read_input_registers(address=5110, count=1, unit=101)
            current_battery = current.registers[0]
            # Prevent integever overflow
            if current.registers[0] < 10000:
                current_battery = current.registers[0]
            elif current.registers[0] > 10000:
                current_battery = current.registers[0]-65535
            return current_battery
        else:
            return 0.0


    def get_battery_voltage(self) -> float:
        """
        Reads actual battery voltage.
        Return value is Battery Voltage. Resolution is 0.1 [V]
        """
        if self.connected:
        # Read battery voltage
            voltage = self.client.read_input_registers(address=5100, count=1, unit=101)
            voltage_battery = voltage.registers[0]
            voltage_battery = float(voltage_battery / 10)
    
            return voltage_battery
        else:
            return 0.0


    def get_dc_link_voltage(self) -> float:
        """
        Reads DC-Link voltage.
        Return value is DC-Link voltage. Resolution is 1.0 [V]
        """
        if self.connected:
            # Read DC link voltage
            voltage = self.client.read_input_registers(address=5127, count=1, unit=101)
            voltage_dc_link = voltage.registers[0]
            return voltage_dc_link
        else:
            return 0.0


    def get_dc_power(self) -> float:
        """
        Reads DC Power.
        Return value is power. Resolution is 1.0 [W]
        """
        if self.connected:
            # Read DC power
            power_dc = self.client.read_input_registers(
                address=5120, count=1, unit=101)
            # watts
            if power_dc.registers[0] < 10000:
                power_dc_value = power_dc.registers[0]
            elif power_dc.registers[0] > 10000:
                power_dc_value = -(65536 - power_dc.registers[0])
            return power_dc_value
        else:
            return 0.0




    def get_active_power(self) -> float:

        """
        Reads active power on all phases, outputs are the sum of power and active power on each phases.
        Resolution is 1.0 [W]. 
        Phases is an array containing the 3 lines power sizes
        """
        if self.connected:
            # Read active power L1
            L1 = self.client.read_input_registers(
                address=5140, count=1, unit=1).registers[0]
            # Read active power L2
            L2 = self.client.read_input_registers(
                address=5141, count=1, unit=1).registers[0]
            # Read active power L3
            L3 = self.client.read_input_registers(
                address=5142, count=1, unit=1).registers[0]
            if L1 > 50000: 
                L1 = L1-65536
            if L2 > 50000:
                L2 = L2-65536
            if L3 > 50000:
                L3 = L3-65536
            
            phases_active = [L1, L2, L3]
                    
            power_active = sum(phases_active)
            
            return phases_active, power_active
        
        else:
            phases_active = [0.0, 0.0, 0.0]
                    
            power_active = 0
            
            return phases_active, power_active



    def get_apparent_power(self) -> float:

        """
        Reads active power on all phases, outputs are the sum of power and active power on each phases.
        Resolution is 1.0 [W]. 
        Phases is an array containing the 3 lines power sizes
        """
        if self.connected:
            L1 = self.client.read_input_registers(
                address=5130, count=1, unit=1).registers[0]
            L2 = self.client.read_input_registers(
                address=5131, count=1, unit=1).registers[0]
            L3 = self.client.read_input_registers(
                address=5132, count=1, unit=1).registers[0]
            phases_apparent = [L1, L2, L3]
            power_apparent = sum(phases_apparent)
            return phases_apparent, power_apparent
        else:
            phases_active = [0.0, 0.0, 0.0]
                    
            power_active = 0
            
            return phases_active, power_active

    def get_status(self):
        
        if self.connected:
            status_dict = {0: 'Power Up', 1: 'Alarm',
                        2: 'Idle', 3: 'Operation', 4: 'Maintenance'}
            
            status = self.client.read_input_registers(address=5000, count=1, unit=0)
            status_msg = status_dict[status.registers[0]]
            return status_msg
        else:
            return 0

    def charge_or_discharge(self):
        
        if self.connected:
            status_dic = {0: 'Charging', 1: 'discharging'}
            direction = self.client.read_holding_registers(address=4006,count=1,unit=0).registers[0]
            pf = self.client.read_holding_registers(address=4217,count=1,unit=0).registers[0]
    
            if (direction == 0 and 1<pf<101) or (direction == 1 and 65535>pf>(65535-101)):
                status = 1
            elif (direction == 1 and 1<pf<101) or (direction == 0 and 65535>pf>(65535-101)):
                status = 0
            status_msg = status_dic[status]
            return status_msg



    def get_current_limiting_status(self) -> int:
        """
        Reads operation mode of trumpf system
        Return variable status can have the following values:
        """
        if self.connected:
            current_dict = {0: 'Inactive', 1: 'Pmax',
                        2: 'Chargemax', 4: 'Dischargemax', 8: 'Vbatmax', 16: 'Vbatmin'}
    
            status = self.client.read_input_registers(address=5123).registers[0]
            current_msg = current_dict[status]
            return current_msg
        else:
            return 0


    def read_TESTER(self) -> str:
        """
        """
        if self.connected:
            status_tester = self.client.read_input_registers(address=5010)
    
            return status_tester    

    def get_error(self):
        """
        Print out if error is in system, return value state descripes status of the system
        """
        if self.connected:
            state = self.client.read_input_registers(address=5000, count=1).registers[0]
            if state == 2:
                print('ERROR')
            else:
                print('No alarm, State of system is = {}'.format(state))
            return state
        else:
            return 0

    

    def get_count_error(self):
        if self.connected:
            error_count = self.client.read_input_registers(address=2809, count=1, unit=0).registers[0]
            return error_count         
        else:
            return 0


    def get_error_msg(self):
        """
        reads the number of errors and returns the error code.
        The number of registers in return value "status" determines the number of errors in the system.
        Please refer to section 9.2 in AC-manuel or 9.3 in DC-manuel for a description of the error code. 
        """
        if self.connected:
            count_error = self.client.read_input_registers(address=2809, count=1, unit=0).registers[0]         
            error = self.client.read_input_registers(address=2810, count=count_error, unit=0)
            return error
        else:
            return 0

    def reset_alarm(self):
        """
        Resets alarm. If BMS timeout has occured, then reset CPU
        """
        if self.connected:
            self.client.write_coil(address=4002, value=1, unit=0)

    def restart_cpu(self):
        """
        Restarts cpu. If BMS timeout has occured, use this function
        """
        if self.connected:
            self.client.write_coil(address=1017, value=1, unit=0)
