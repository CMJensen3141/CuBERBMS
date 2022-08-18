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
from bms_registers import *

import msvcrt

write_is_allowed = True;
counter =0;

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
        return [status,np.zeros([1,num_to_read])]
        
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
            

client = ModbusClient('localhost', retries=3, port=5020)            

def run_sync_client():

    
    # Client setup and Modbus parameters
    # ------------------------------------------------------------------------#
    # client = ModbusClient('localhost', retries=3, port=5020)
    
    print("Opening connection to Modbus client")
    client.connect() # Connect to the modbus server
    UNIT = 0x1 # Logic unit number
    # client.write_register(Reg_Ctrl_BMS, 1, unit=UNIT)
    # rr = client.read_holding_registers(Reg_Ctrl_BMS, 1, unit=UNIT)
    # print(rr.registers)
    SM = state_machine(client,UNIT) # Instantiate the state machine
    datalist = read_multiple_registers(HOLDING_REGISTER_COUNT, client, UNIT)
    print("Connection successfully opened")
 
    
    # ----------------------------------------------------------------------- #
    # Run one full charging (0.95 SOC) and full discharge (0.05 SOC) cycle
    # ----------------------------------------------------------------------- #
 
    while(SM.running() == True):
        # print("Entering the Matrix")
        SM.state_handler(client)
        # time.sleep(1)
        # datalist = np.append(datalist, read_multiple_registers(HOLDING_REGISTER_COUNT, client, UNIT), axis=0)
    
    # ----------------------------------------------------------------------- #
    # close the client
    # ----------------------------------------------------------------------- #
    print("Closing connection to BMS server")
    client.close()
    print("Connection successfully closed")


if __name__ == "__main__":

    run_sync_client()

#    test=easygui.ynbox('Run test?', 'Title', ('Yes', 'No'))
#    if test:
#        #print("yn={}".format(test))
#        run_sync_client()
#    else:
#        print("Test cancelled.")
