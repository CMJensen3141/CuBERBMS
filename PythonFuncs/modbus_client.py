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
# from pymodbus.client.sync import ModbusUdpClient as ModbusClient
# from pymodbus.client.sync import ModbusSerialClient as ModbusClient

# --------------------------------------------------------------------------- #
# configure the client logging
# --------------------------------------------------------------------------- #
#import logging
#FORMAT = ('%(asctime)-15s %(threadName)-15s '
#          '%(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
#logging.basicConfig(format=FORMAT)
#log = logging.getLogger()
#log.setLevel(logging.DEBUG)

#import easygui

import time
import math

import numpy as np
import matplotlib.pyplot as plt
import pylab as pl
#import numpy as np
#import matplotlib.pyplot as plt
#import pylab as pl
#from scipy.interpolate import griddata

import msvcrt
#from bms_registers import *
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

BMS_STANDBY = 0
BMS_CHARGING = 1
BMS_DISCHARGING = 2

def readvalue(client, registernumber, inputarray):
    UNIT = 0x1
    rr = client.read_holding_registers(registernumber, 1, unit=UNIT)
    if rr.function_code >= 0x80:
        print("Read error")
        inputarray=np.append(inputarray, -1.0)
    else:
        #print("Read ok ")
        ##print("register = {}, value = {}".format(Reg_Ctrl_BMS,rr.registers[0]))
        inputarray=np.append(inputarray, rr.registers[0]*1.0)
            
    return (inputarray)
    
def Read_Registers(num_to_read, client, UNIT):
    try:
        #time.sleep(1)
        rr = client.read_holding_registers(Reg_Ctrl_BMS, num_to_read, unit=UNIT)
        if rr.function_code >= 0x80:
            print("Read error")
        else:
            if rr: 
                #print (rr.registers)
                regs = np.array(rr.registers)
                regs = regs.reshape(1,len(regs))
                #print (regs)
                
                #nprow = ConvertToNpRow( rr.registers, num_registers_to_read)
                
                print("Read OK! Register = {}, value = {}".format(Reg_Ctrl_BMS,rr.registers[0]))
    except:
        print ('(exception detected)')
    if rr:
        return regs
    else:
        return np.zeros(1,num_to_read)

def run_sync_client():
    #    client = ModbusClient('localhost', retries=3, retry_on_empty=True)
    # ------------------------------------------------------------------------#
    client = ModbusClient('localhost', retries=3, port=5020)
    
    #client = ModbusClient('192.168.1.2', retries=3, port=502) #Trumpf inverter
    
    # from pymodbus.transaction import ModbusRtuFramer
    # client = ModbusClient('localhost', port=5020, framer=ModbusRtuFramer)
    # client = ModbusClient(method='rtu', port='/dev/ptyp0', timeout=1,
    #                       baudrate=9600)
    client.connect()
    register_to_write = 100;
    num_measures = 100;
#    power = np.zeros(0)
#    power = np.append(power, 1.0)
#    power = np.append(power, 2.0)
#    power = np.append(power, 3.0)
#    print("{}".format(power))   
    UNIT = 0x1

    datalist = Read_Registers(HOLDING_REGISTER_COUNT, client, UNIT)

    #log.debug("Write to a holding register and read back")
    
    try:
        rr = client.read_holding_registers(Reg_Ctrl_BMS, 1, unit=UNIT)
        if rr.function_code >= 0x80:
            print("Read error")
#        else:
#            print("Read ok ")
#            print("register = {}, value = {}".format(Reg_Ctrl_BMS,rr.registers[0]))
        if write_is_allowed:
#            print("writing!")
            value_to_write = 1;
#            print("register = {}, value = {}".format(Reg_Ctrl_BMS,value_to_write))
            rq = client.write_register(Reg_Ctrl_BMS, value_to_write, unit=UNIT)
            rq = client.write_register(Reg_P_ref, 1000, unit=UNIT)

        rr = client.read_holding_registers(Reg_Ctrl_BMS, 1, unit=UNIT)

        if rr.function_code >= 0x80:
            print("Read error")
        else:
            dummy = 2;
#            print("Read ok ")
#            print("register = {}, value = {}".format(Reg_Ctrl_BMS,rr.registers[0]))
            
#        if write_is_allowed:
#            print("writing!")
    except:
        print ('(exception detected)')

    counter = 0;
    while (counter < 3):
        counter = counter + 1
        datalist = np.append(datalist, Read_Registers(HOLDING_REGISTER_COUNT, client, UNIT), axis=0)
        print (datalist)        

    print ("Ikke doed 1")
    if write_is_allowed:
#        print("writing!")
        value_to_write = 2;
#        print("register = {}, value = {}".format(Reg_Ctrl_BMS,value_to_write))
        rq = client.write_register(Reg_Ctrl_BMS, value_to_write, unit=UNIT)
        rq = client.write_register(Reg_P_ref, 1000, unit=UNIT)
    print ("Ikke doed 1")
    counter = 0;
    while (counter < 50):
        counter = counter + 1
        print ("Ikke doed 2")
        
        try:
            time.sleep(1)
            rr = client.read_holding_registers(Reg_SOC, 1, unit=UNIT)
            if rr.function_code >= 0x80:
                print("Read error")
            #else:
            #    print("Read OK! Register = {}, value = {}".format(Reg_SOC,rr.registers[0]))
            rr = client.read_holding_registers(Reg_Stack_voltage, 1, unit=UNIT)

            if rr.function_code >= 0x80:
                print("Read error")
            else:
                print("Read OK! stack voltage = {}, value = {}".format(Reg_Stack_voltage,rr.registers[0]))
        except:
            print ('(exception)')

    if write_is_allowed:
        print("writing!")
        value_to_write = 0;
        print("register = {}, value = {}".format(Reg_Ctrl_BMS,value_to_write))
        rq = client.write_register(Reg_Ctrl_BMS, value_to_write, unit=UNIT)
#        value_to_write = 22332;
#        print("register = {}, value = {}".format(register_to_write,value_to_write))
#        rq = client.write_register(register_to_write, value_to_write, unit=UNIT)
    
    #assert(not rq.isError())     # test that we are not an error
    #assert(rr.registers[0] == 10)       # test the expected value
 
    # ----------------------------------------------------------------------- #
    # close the client
    # ----------------------------------------------------------------------- #
    client.close()


if __name__ == "__main__":

    run_sync_client()

#    test=easygui.ynbox('Run test?', 'Title', ('Yes', 'No'))
#    if test:
#        #print("yn={}".format(test))
#        run_sync_client()
#    else:
#        print("Test cancelled.")
