# -*- coding: utf-8 -*-
"""
Created on Fri Feb 19 13:29:57 2021

@author: au81602
"""

#!/usr/bin/env python
"""
Pymodbus Synchronous Client Examples
--------------------------------------------------------------------------

The following is an example of how to use the synchronous modbus client
implementation from pymodbus.

It should be noted that the client can also be used with
the guard construct that is available in python 2.5 and up::

    with ModbusClient('127.0.0.1') as client:
        result = client.read_coils(1,10)
        print result
"""
# --------------------------------------------------------------------------- #
# import the various server implementations
# --------------------------------------------------------------------------- #
from pymodbus.client.sync import ModbusTcpClient as ModbusClient


import time
import math

import numpy as np
import matplotlib.pyplot as plt
import pylab as pl
from modbus_class import Client


import msvcrt

Reg_Ctrl_BMS 				= 100; #Control of the battery (charge or discharge)
Reg_SOC						= 101; #State of charge of the battery
Reg_P_ref					= 102; #Reference for charge level (positive = discharging from battery to grid)
Reg_GaussVolt				= 103; #Gaussian charge voltage on surface of tank
Reg_SOC_Gauss				= 104; #State of charge estimation based on Gaussian charge voltage
Reg_BMS_Error				= 105; #Errors from BMS (0= no error)
Reg_Stack_Current			= 106; #Current in the stack of the CuRFB
Reg_Stack_voltage			= 107; #Voltage in the stack of the CuRFB
Reg_Chlorine				= 108; #Formation of Chlorine due to overvoltage
Reg_Temp_anolyte			= 109; #Temperature in the anolyte tank
Reg_Temp_anolyte_In			= 110; #Temperature at the entrance of the stack
Reg_Flow_Anolyte			= 111; #Flow of the anolyte circuit
Reg_Pressure_Anolyte		= 112; #Pressure in the anolyte circuit
Reg_Speed_Anolyte			= 113; #Pump speed for the anolyte circuit
Reg_Temp_catholyte			= 114; #Temperature in the catholyte tank
Reg_Temp_catholyte_In		= 115; #Temperature at the entrance of the stack
Reg_Flow_catholyte			= 116; #Flow of the catholyte circuit
Reg_Pressure_catholyte		= 117; #Pressure in the catholyte circuit
Reg_Speed_catholyte			= 118; #Pump for the catholyte circuit
Reg_DC_DC_ON				= 119; #DC/DC converter (on/off)
Reg_DC_DC_CURRENT			= 120; #Charging/discharging current in DC/DC converter
Reg_DC_DC_VOLTAGE			= 121; #Charging voltage/cell voltage in DC/DC converter
Reg_DC_DC_POWER				= 122; #Charging power in DC/DC converter (positive = discharing)
Reg_DC_DC_TEMP				= 123; #Internal temperature in DC/DC converter
Reg_DC_DC_AMBIENT_TEMP		= 124; #Ambient temperature in DC/DC converter
Reg_DC_DC_ERRORS			= 125; #Errors from DC/DC converter (0 = no error)
Reg_AC_DC_ON				= 126; #AC/DC converter (on/off)
Reg_AC_DC_GRID_CURRENT		= 127; #AC/DC converter current (positive = discharging)
Reg_AC_DC_GRID_VOLTAGE		= 128; #Grid voltage in AC/DC converter (phase to phase)
Reg_AC_DC_GRID_POWER		= 129; #Grid power in AC/DC converter (positive = output)
Reg_DC_LINK_VOLTAGE			= 130; #DC link voltage between AC/DC and DC/DC converters
Reg_AC_DC_TEMP				= 131; #Internal temperature in AC/DC converter
Reg_AC_DC_AMBIENT_TEMP		= 132; #Ambient temperature in AC/DC converter
Reg_AC_DC_ERRORS			= 133; #Errors from AC/DC converter (0 = no error)
Reg_Speed_Thermal			= 134; #Pump speed for thermal circuit
Reg_T_Thermal_tank			= 135; #Temperature in the thermal tank
Reg_Simulation_mode			= 136; #0=Live version on real HW, 1=Simulation level 1 (forward Euler, 1. order)
Reg_Time_contraction		= 137; #Multiplier for time in simulation (speed up factor), reduces accuracy of simulation
Reg_Simulation_clock		= 138; #Internal simulation in seconds
Reg_thermal_temp_catholyte  = 139; #Temperature of the heat exchanger in the catholyte circuit
Reg_thermal_temp_anolyte    = 140; #Temperature of the heat exchanger in the anolyte circuit
reg_anolyte_pressure_out    = 141; #Pressure in the anolyte circuit after the stack
reg_catholyte_pressure_out  = 142; #Pressure in the catholyte circuit after the stack
reg_anolythe_temp_out       = 143; #Temperature in the anolyte circuit after the stack
reg_catholyte_temp_out      = 144; #catholyte_temp_out
reg_control_bms             = 145; #BMS reference mode, 1 = power reference, 0= voltage reference/max current (Visblue control method) 
reg_rs_est                  = 146; # serial resistance * 1000 
reg_rp_est                  = 147; 
reg_cp_hi_est               = 148; # cp >> 15
reg_cp_lo_est               = 149; # cp & ox7fff
reg_ocv_est                 = 150; # ocv * 100

HOLDING_REGISTERS           = 139;
HOLDING_REGISTER_START      = 99;
HOLDING_REGISTER_END        = 150;
HOLDING_REGISTER_MAX        = HOLDING_REGISTER_START + HOLDING_REGISTERS ;
HOLDING_REGISTER_COUNT      = HOLDING_REGISTER_END - HOLDING_REGISTER_START -1;
HOLDING_REGISTER_COUNT = 44;

write_is_allowed = True;
counter =0;

# Define BMS states
BMS_STANDBY = 0
BMS_CHARGING = 1
BMS_DISCHARGING = 2

# Define state machine states
STATE_IDLE = 0
STATE_START = 1
STATE_CHARGE = 2
STATE_SHUTDOWN = -1
STATE_DISCHARGE = -2
STATE_PAUSE = 3
STATE_ERROR = 666

def readvalue(client, registernumber, inputarray):
    UNIT = 0x1
    rr = client.read_holding_registers(registernumber, 1, unit=UNIT)
    if rr.function_code >= 0x80:
        print("Read error")
        inputarray=np.append(inputarray, -1.0)
    else:
        inputarray=np.append(inputarray, rr.registers[0]*1.0)
            
    return (inputarray)
    
def read_multiple_registers(num_to_read, client, UNIT):
    status = False
    try:
        rr = client.read_holding_registers(Reg_Ctrl_BMS, num_to_read, unit=UNIT)
        if rr.function_code >= 0x80:
            print("Read error")
        else:
            if rr: 
                regs = np.array(rr.registers)
                regs = regs.reshape(1,len(regs))
                status = True
    except:
        print ('(exception detected)')
    if status == True:
        return [status,regs]
    else:
        return [status,np.zeros(1,num_to_read)]
        
def read_single_register(client,UNIT,regnum):
    status = True
    value = 0
    try:
        rr = client.read_holding_registers(regnum, 1, unit=UNIT)
        if rr.function_code >= 0x80:
            print("Read error")
            status = False
        else:
            value = rr.registers[0]
    except:
        print ('(exception detected)')
        status = False
    
    return [status, value] 

def write_single_register(client,UNIT,regnum,value):
    status = True
    try:
        rq = client.write_register(regnum, value, unit=UNIT)
        if rq.function_code >= 0x80:
            print("Write error")
            status = False
    except:
        print ('(exception detected)')
        status = False
    
    return status 

class state_machine:
    def __init__(self,client,unit):
        self.state = STATE_IDLE
        self.client = client
        self.unit = unit
        self.waitcounter = 0
        self.run = True
    
    def running(self):
        return self.run
    
    def state_handler(self,client):
        if self.state == STATE_IDLE:
            self.state = STATE_START
            self.run = True
                
            
        elif self.state == STATE_START:
            rq = write_single_register(client,self.unit,Reg_Ctrl_BMS, BMS_CHARGING)
            if rq == True:
                self.state = STATE_CHARGE
            else:
                self.state = STATE_ERROR
                print("Error in start sequence")
                return False
            rq = write_single_register(client,self.unit,Reg_P_ref, 2500)
            if rq == True:
                self.state = STATE_CHARGE
            else:
                self.state = STATE_ERROR
                print("Error in start sequence")
                return False
            res = read_single_register(self.client,self.unit,Reg_Ctrl_BMS)
            if res[0] == True:
                print(str(res[1]))
            else:
                print("Failed to read and echo")
            
        elif self.state == STATE_CHARGE:
            res = read_single_register(self.client,self.unit,Reg_BMS_Error)
            if res[0] == True:
                if res[1] != 0:
                    print("Encountered an error with the BMS")
                    self.state = STATE_ERROR
                    return False
            else:
                self.state = STATE_ERROR
                print("Read error A during charging")
                return False
            soc = read_single_register(self.client,self.unit,Reg_SOC)
            if soc[0] == True:
                print(str(soc[1]))
                if soc[1] >= 50:
                    if not write_single_register(self.client,self.unit,Reg_Ctrl_BMS, BMS_STANDBY):
                        self.state = STATE_ERROR
                        print("Write error during charging")
                    else:
                        self.state = STATE_PAUSE
                        self.waitcounter = 1
            else:
                self.state = STATE_ERROR
                print("Read error B in during charging")
                return False
            

        elif self.state == STATE_PAUSE:
            if self.waitcounter > 0:
                print("Pausing for " + str(self.waitcounter) + " more seconds")
                self.waitcounter = self.waitcounter - 1
            else:
                rq = write_single_register(client,self.unit,Reg_Ctrl_BMS, BMS_DISCHARGING)
                if rq == True:
                    self.state = STATE_DISCHARGE
                else:
                    self.state = STATE_ERROR
                    print("Error in transition to discharge")
                    return False
                
        elif self.state == STATE_DISCHARGE:
            res = read_single_register(self.client,self.unit,Reg_BMS_Error)
            if res[0] == True:
                if res[1] != 0:
                    self.state = STATE_ERROR
                    print("Read error A during discharge")
                    return False
            else:
                self.state = STATE_ERROR
                return False
            soc = read_single_register(self.client,self.unit,Reg_SOC)
            print("SOC is " + str(soc[1]))
            if soc[0] == True:
                if soc[1] <= 5:
                    if not write_single_register(self.client,self.unit,Reg_Ctrl_BMS, BMS_STANDBY):
                        self.state = STATE_ERROR
                        print("Write error during discharge")
                        return False
                    else:
                        self.state = STATE_SHUTDOWN
                        self.waitcounter = 1
            else:
                self.state = STATE_ERROR
                print("Read error B during discharge")
                return False
            
        elif self.state == STATE_SHUTDOWN:
            self.run = False
            
        elif self.state == STATE_ERROR:
            write_single_register(self.client,self.unit,Reg_Ctrl_BMS, BMS_STANDBY)
            self.run = False
            
        else:
            self.state = STATE_ERROR



def SetConverterPower(self,ref):
    self.connect_to_client()
    
    self.set_constant_power(ref)
    
    self.disconnect_from_client()

def GetConverterPower(self):
    self.connect_to_client()
    
    [phases, power] = self.get_apparent_power()

    self.disconnect_from_client()

    return power


            
# client = ModbusClient('localhost', retries=3, port=5020) 
client = Client(ip_address='192.168.1.17')       

def run_sync_client():

    
    # Client setup and Modbus parameters
    # ------------------------------------------------------------------------#
    # client = ModbusClient('localhost', retries=3, port=5020)
    
    print("Opening connection to Trumpf converter")
    [converter, connected] = client.connect_to_client() # Connect to the converter server
    print(connected)
    # converter = mbl.Client(ip_address='192.168.1.17')
    # connected = converter.connect_to_client()
    converter.discharge_battery()
    converter.start_transmission(1000)
    time.sleep(10)
    print(str(converter.get_active_power()))
    converter.set_AC_power(110)
    time.sleep(10)
    print(str(converter.get_active_power()))
    time.sleep(10)
    converter.stop_transmission()
    converter.disconnect_from_client()


if __name__ == "__main__":

    run_sync_client()

#    test=easygui.ynbox('Run test?', 'Title', ('Yes', 'No'))
#    if test:
#        #print("yn={}".format(test))
#        run_sync_client()
#    else:
#        print("Test cancelled.")
