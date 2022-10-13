# --------------------------------------------------------------------------- #
# import the hydraulic and thermal model implementations
# --------------------------------------------------------------------------- #

import random
import CuBMS as CB
import numpy as np
import ControlFuncs as CF
import RFB_CombinedModel as ComMod
from UnitConversions import *


# --------------------------------------------------------------------------- #
# import the various server implementations
# --------------------------------------------------------------------------- #
from pymodbus.server.sync import StartTcpServer, ModbusTcpServer

from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSparseDataBlock, ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer

from twisted.internet.task import LoopingCall
from twisted.internet import reactor
import threading
# --------------------------------------------------------------------------- #
# configure the service logging
# --------------------------------------------------------------------------- #
import logging

from bms_registers import *
import RFB_BMS as BMS
AtmPres = 1000


logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.ERROR)


# --------------------------------------------------------------------------- #
# Precalculations for battery and energy
# --------------------------------------------------------------------------- #

active_BMS = BMS.Sim_BMS()

dummyvalue = 15.0

def int15(floatvalue):
    value = int(floatvalue + 0.5)
    if value < 0:
        value =0
    elif value >32767:
        value=32767
    return int(value)
        
def randvalue(minrange, maxrange):
    return random.randint(minrange, maxrange)
        
#Call back functions reacieve current value in register and return value is written back to the register
def Read_Ctrl_BMS          (Value):
    return int15(active_BMS.get_BMS_state());
def Read_SOC               (Value):
    return int15(active_BMS.get_battery_soc());
def Read_P_ref             (Value):
	return int15(active_BMS.get_power_reference());
def Read_GaussVolt         (Value):
	return (randvalue(0,100));            ###@TODO
def Read_SOC_Gauss         (Value):
	return (randvalue(0,100));            ###@TODO
def Read_BMS_Error         (Value):
	return 0;            ###@TODO
def Read_Stack_Current     (Value):
	return int15(active_BMS.get_battery_current());
def Read_Stack_voltage     (Value):
	return int15(active_BMS.get_battery_voltage());
def Read_Chlorine          (Value):
	return (randvalue(0,30000));            ###@TODO
def Read_Temp_anolyte      (Value):
    out = active_BMS.battery.CombinedModel.GetTemps()
    return int15(out[0])
def Read_Temp_anolyte_In   (Value):
    out = active_BMS.battery.CombinedModel.GetTemps()
    return int15(out[1]);            ###@TODO
def Read_Flow_Anolyte      (Value):
    return int15(active_BMS.battery.get_flowrate()) # Check that units are correct (currently l/s)
def Read_Pressure_Anolyte  (Value):
    return int15(AtmPres-active_BMS.battery.CombinedModel.GetAnoPressure_Pump()); # Check that units are correct (currently mbar)
def Read_Speed_Anolyte     (Value):
    return int15(active_BMS.get_positive_pump_speed());
def Read_Temp_catholyte    (Value):
    out = active_BMS.battery.CombinedModel.GetTemps()
    return int15(out[0])
def Read_Temp_catholyte_In (Value):
    out = active_BMS.battery.CombinedModel.GetTemps()
    return int15(out[2])            
def Read_Flow_catholyte    (Value):
	return int15(active_BMS.battery.CombinedModel.GetCatFlows())             # Check that units are correct (currently l/s)
def Read_Pressure_catholyte(Value):
	return int15(AtmPres-active_BMS.battery.CombinedModel.GetCatPressure_Pump()); # Check that units are correct (currently mbar)
def Read_Speed_catholyte   (Value):
	return int15(active_BMS.get_positive_pump_speed());
def Read_DC_DC_ON          (Value):
	return int15(active_BMS.battery.Inverter.get_inverter_state());
def Read_DC_DC_CURRENT     (Value):
	return int15(active_BMS.battery.Inverter.get_DC_current());
def Read_DC_DC_VOLTAGE     (Value):
	return int15(active_BMS.battery.Inverter.get_DC_voltage());
def Read_DC_DC_POWER       (Value):
	return int15(active_BMS.battery.Inverter.get_DC_power());
def Read_DC_DC_TEMP        (Value):
	return int15(active_BMS.battery.Inverter.get_inverter_internal_temp());
def Read_DC_DC_AMBIENT_TEMP(Value):
	return int15(active_BMS.battery.Inverter.get_inverter_ambient_temp());
def Read_DC_DC_ERRORS      (Value):
	return (randvalue(0,1));            ###@TODO
def Read_AC_DC_ON          (Value):
	return int15(active_BMS.battery.Inverter.get_inverter_state());
def Read_AC_DC_GRID_CURRENT(Value):
	return int15(active_BMS.battery.Inverter.get_grid_current());
def Read_AC_DC_GRID_VOLTAGE(Value):
	return int15(active_BMS.battery.Inverter.get_grid_voltage());
def Read_AC_DC_GRID_POWER  (Value):
	return int15(active_BMS.battery.get_stack_power());
def Read_DC_LINK_VOLTAGE   (Value):
	return int15(active_BMS.battery.Inverter.get_DC_voltage());
def Read_AC_DC_TEMP        (Value):
	return int15(active_BMS.battery.Inverter.get_inverter_internal_temp());
def Read_AC_DC_AMBIENT_TEMP(Value):
	return int15(active_BMS.battery.Inverter.get_inverter_ambient_temp());
def Read_AC_DC_ERRORS      (Value):
	return (0);            ###@TODO
def Read_Speed_Thermal     (Value):
	return (randvalue(2800,2999));            ###@TODO
def Read_T_Thermal_tank    (Value):
	return (randvalue(23,25));            ###@TODO
def Read_Simulation_mode   (Value):
	return (Value);            ###@TODO
def Read_Time_contraction  (Value):
	return int15(active_BMS.battery.get_speedup_factor());
def Read_Simulation_clock  (Value):
	return (Value);
def Read_thermal_temp_catholyte (value):
    out = active_BMS.battery.CombinedModel.GetTemps()
    return int15(out[0])   
def Read_thermal_temp_anolyte   (value):
    out = active_BMS.battery.CombinedModel.GetTemps()
    return int15(out[0])      
def Read_anolyte_pressure_out   (value):
	return int15(AtmPres-active_BMS.battery.CombinedModel.GetAnoPressure_Stack());      
def Read_catholyte_pressure_out (value):
	return int15(AtmPres-active_BMS.battery.CombinedModel.GetCatPressure_Stack());      
def Read_anolyte_temp_out      (value):
	return (randvalue(23,25));      
def Read_catholyte_temp_out     (value):
	return (randvalue(23,25));      
def Read_Rs_est    (self):
    return int15(active_BMS.Rs * 1000)
def Read_Rp_est    (self):
    return int15(active_BMS.Rp * 100)
def Read_Cp_hi_est (self):
    return int15(int15(active_BMS.Cp) >> 15)
def Read_CP_lo_est (self):
    return int15(int15(active_BMS.Cp) & 0x7fff)
def Read_OCV_est   (self):
    return int15(active_BMS.OCV * 100);
def Read_Load_Ref  (self):
    return int15(active_BMS.battery.get_load_ref())


read_function_switch_statement = {
	Reg_Ctrl_BMS 				:Read_Ctrl_BMS 				,
	Reg_SOC						:Read_SOC					,
	Reg_P_ref					:Read_P_ref					,
	Reg_GaussVolt				:Read_GaussVolt				,
	Reg_SOC_Gauss				:Read_SOC_Gauss				,
	Reg_BMS_Error				:Read_BMS_Error				,
	Reg_Stack_Current			:Read_Stack_Current			,
	Reg_Stack_voltage			:Read_Stack_voltage			,
	Reg_Chlorine				:Read_Chlorine				,
	Reg_Temp_anolyte			:Read_Temp_anolyte			,
	Reg_Temp_anolyte_In			:Read_Temp_anolyte_In		,
	Reg_Flow_Anolyte			:Read_Flow_Anolyte			,
	Reg_Pressure_Anolyte		:Read_Pressure_Anolyte		,
	Reg_Speed_Anolyte			:Read_Speed_Anolyte			,
	Reg_Temp_catholyte			:Read_Temp_catholyte		,	
	Reg_Temp_catholyte_In		:Read_Temp_catholyte_In		,
	Reg_Flow_catholyte			:Read_Flow_catholyte		,	
	Reg_Pressure_catholyte		:Read_Pressure_catholyte	,	
	Reg_Speed_catholyte			:Read_Speed_catholyte		,
	Reg_DC_DC_ON				:Read_DC_DC_ON				,
	Reg_DC_DC_CURRENT			:Read_DC_DC_CURRENT			,
	Reg_DC_DC_VOLTAGE			:Read_DC_DC_VOLTAGE			,
	Reg_DC_DC_POWER				:Read_DC_DC_POWER			,
	Reg_DC_DC_TEMP				:Read_DC_DC_TEMP			,	
	Reg_DC_DC_AMBIENT_TEMP		:Read_DC_DC_AMBIENT_TEMP	,	
	Reg_DC_DC_ERRORS			:Read_DC_DC_ERRORS			,
	Reg_AC_DC_ON				:Read_AC_DC_ON				,
	Reg_AC_DC_GRID_CURRENT		:Read_AC_DC_GRID_CURRENT	,	
	Reg_AC_DC_GRID_VOLTAGE		:Read_AC_DC_GRID_VOLTAGE	,	
	Reg_AC_DC_GRID_POWER		:Read_AC_DC_GRID_POWER		,
	Reg_DC_LINK_VOLTAGE			:Read_DC_LINK_VOLTAGE		,
	Reg_AC_DC_TEMP				:Read_AC_DC_TEMP			,	
	Reg_AC_DC_AMBIENT_TEMP		:Read_AC_DC_AMBIENT_TEMP	,	
	Reg_AC_DC_ERRORS			:Read_AC_DC_ERRORS			,
	Reg_Speed_Thermal			:Read_Speed_Thermal			,
	Reg_T_Thermal_tank			:Read_T_Thermal_tank        ,		
    Reg_Simulation_mode         :Read_Simulation_mode       ,
    Reg_Time_contraction        :Read_Time_contraction      ,
    Reg_Simulation_clock        :Read_Simulation_clock      ,
    Reg_thermal_temp_catholyte  :Read_thermal_temp_catholyte,
    Reg_thermal_temp_anolyte    :Read_thermal_temp_anolyte  ,
    Reg_anolyte_pressure_out    :Read_anolyte_pressure_out  ,
    Reg_catholyte_pressure_out  :Read_catholyte_pressure_out,
    Reg_anolyte_temp_out        :Read_anolyte_temp_out     ,
    Reg_catholyte_temp_out      :Read_catholyte_temp_out    ,
    Reg_rs_est                  :Read_Rs_est, 
    Reg_rp_est                  :Read_Rp_est, 
    Reg_cp_hi_est               :Read_Cp_hi_est,
    Reg_cp_lo_est               :Read_CP_lo_est,
    Reg_ocv_est                 :Read_OCV_est,
    Reg_Load_Ref                :Read_Load_Ref
    };

        
        
#def read_fun_1():
#    print ('R_Fun 1 ')
#    return (1);
#def read_fun_2():
#    print ('R_Fun 2 ')
#    return (2);
#def read_fun_3():
#    print ('R_Fun 3 ')
#    return (3);
#
#def write_fun_1(value):
#    print ('W_Fun 1 ({})'.format(value))
#    return (101);
#def write_fun_2(value):
#    print ('W_Fun 2 ({})'.format(value))
#    return (102);

"""
	class pymodbus.datastore.ModbusSequentialDataBlock(address, values)
	Bases: pymodbus.datastore.store.BaseModbusDataBlock
    Creates a sequential modbus datastore
    classmethod create()
        Factory method to create a datastore with the full address space initialized to 0x00
        Returns:	An initialized datastore

    getValues(address, count=1)
        Returns the requested values of the datastore
        Parameters:	
            address – The starting address
            count – The number of values to retrieve
        Returns:	
			The requested values from a:a+c

    setValues(address, values)
        Sets the requested values of the datastore
        Parameters:	
            address – The starting address
            values – The new values to be set

    validate(address, count=1)
        Checks to see if the request is in range
        Parameters:	
            address – The starting address
            count – The number of values to test for
        Returns:	
			True if the request in within range, False otherwise
"""
class CustomDataBlock(ModbusSparseDataBlock):
    """ A datablock that stores the new value in memory
    and performs a custom action after it has been stored.
    """
        
    def getValues(self, address, count=1):
        ''' Returns the requested values of the datastore
        :param address: The starting address
        :param count: The number of values to retrieve
        :returns: The requested values from a:a+c
        '''
        #with self.rwlock.get_reader_lock():
        for idx in range(count):
            current_address =  address+idx;
            current_value = super(CustomDataBlock, self).getValues(address+idx,1)[0]
            if (current_address >= HOLDING_REGISTER_START):
                if (current_address <= HOLDING_REGISTER_MAX):
                    current_value = int(read_function_switch_statement[current_address-1](current_value))
                else:
                    current_value = 0
            else:
                current_value = 0
            super(CustomDataBlock, self).setValues(current_address, int(current_value)) 
            # print("Read {} at {} ".format(current_value , current_address))
        return super(CustomDataBlock, self).getValues(address, count)
    def setValues(self, address, value):
        """ Sets the requested values of the datastore

        :param address: The starting address
        :param values: The new values to be set
        """
        #@TODO: Writeable registers need to be updated: check if any missing!
        super(CustomDataBlock, self).setValues(address, value)

        # whatever you want to do with the written value is done here,
        # however make sure not to do too much work here or it will
        # block the server, espectially if the server is being written
        # to very quickly
        # print("Made it this far in the write sequence!")
        for idx in range(len(value)):
            current_address = (address+idx) - 1
            if current_address == Reg_Ctrl_BMS:
                # print("Made it to the part where I write to the BMS state")
                active_BMS.set_BMS_state(value[idx])
                print("wrote {} at {} (BMS control)".format(value[idx], address+idx))                
            elif current_address ==  Reg_P_ref:
                active_BMS.set_power_reference(float (value[idx]))
                print("wrote {} at {} (p_ref)".format(value[idx], address+idx))
            elif current_address ==  Reg_Time_contraction:
                active_BMS.battery.set_speedup_factor(float (value[idx]))
                print("wrote {} at {} (p_ref)".format(value[idx], address+idx))    
            elif current_address == Reg_Load_Ref:
                active_BMS.battery.set_load_ref(float (value[idx]))
                print("wrote {} at {} (Load Reference)".format(value[idx], address+idx)) 
            else:
                print("<wrote {} at {} >".format(value[idx], address+idx))


def updatevalues(a):
    #print("------------START----------")
    # contxt = a[1]

    active_BMS.update_simulation()
#    print(".", end=" ")
#    rfuncode = 3
#    wfuncode = 16
#    slave_id = 0x01
#    address = Reg_Simulation_clock
#    contxt = a.context[slave_id]
#    values = contxt.getValues(rfuncode, address, count=1)
#    for idx in range(len(values)):
#        values[idx] = values[idx]+1
#        if values[idx] > 65536: 
#             values[idx] =0
#
#    contxt.setValues(wfuncode, address, values)
#    #print("-------------END-------------")

def run_server():
    # ----------------------------------------------------------------------- #
    # initialize your data store
    # ----------------------------------------------------------------------- #
    #   di=block, co=block, hr=block, ir=block
    # We only need holding register
    holdingregister_block  = CustomDataBlock([1]*HOLDING_REGISTER_MAX)
    store_1 = ModbusSlaveContext(di=None, co=None, hr=holdingregister_block, ir=None)

    slaves = {
        0x01: store_1,
    }

    context = ModbusServerContext(slaves=slaves, single=False)

    # ----------------------------------------------------------------------- #
    # initialize the server information
    # ----------------------------------------------------------------------- #
    # If you don't set this or any fields, they are defaulted to empty strings.
    # ----------------------------------------------------------------------- #
    identity = ModbusDeviceIdentification()
    identity.VendorName = 'Cuber'
    identity.ProductCode = 'Cuber BMS PC-SW'
    identity.VendorUrl = 'https://www.cuberproject.com/'
    identity.ProductName = 'Cuber BMS Server'
    identity.ModelName = 'BMS Server'
    identity.MajorMinorRevision = '0.2'

    # ----------------------------------------------------------------------- #
    # run the server you want
    # ----------------------------------------------------------------------- #
    # Tcp:
    # server = StartTcpServer(context, identity=identity, address=('0.0.0.0', 255))
    # start server in a separate thread so that is not blocking
    # server.start_server()

    # to access the blocks for slave 1
    # store_1=server.context[1]

    # to read from the block
    # print("------")
    # print(store_1.getValues(4,0,32))

    # to write to the block
    # store_1.setValues(0x0F, 0, [111, 121, 122, 123, 124])

    interval = 0.05

    server = ModbusTcpServer(context, identity=identity, address=('0.0.0.0', 5020))
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    # reactor.run()
    loop = LoopingCall(f=updatevalues, a=server)
    loop.start(interval, now=True)
    print("")
    print("")
    print("------------------------------------------------------------")
    print("Server starting...")
    reactor.run()

#!/usr/bin/python

if __name__ == "__main__":
        run_server()
    
