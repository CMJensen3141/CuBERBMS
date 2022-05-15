# --------------------------------------------------------------------------- #
# import the hydraulic and thermal model implementations
# --------------------------------------------------------------------------- #

import RFB_ThermalModel as TModel
import RFB_LumpedHydraulicModel as HModel
import random
import CuBMS as CB
import numpy as np
import ControlFuncs as CF



# --------------------------------------------------------------------------- #
# set invariant parameters of imported models
# --------------------------------------------------------------------------- #

# thermal parameters
R_stack = 2.1e-04
R_pipes = 1.0e-03
R_he = 3.8e-03
R_ambient = 2*8.4e-03
C_stack = 4.7e03
C_pipes = 5.2e04
C_he = 4.7e05

# hydraulic parameters
viscosity = 0.006 # Electrolyte viscosity
permeability = 1.685e-10 # Electrode permeability
density = 1400
length = 0.26 # Half-cell length
width = 0.30 # Half-cell width
depth = 3e-03 # Half-cell depth
Nseries = 15 # Number of stack cells in series

# BMS parametre
BMS_mu     = 0.1 #Step size i RLS estimator
BMS_Lambda = 0.9999 #Forgetting factor in RLS
BMS_EstMode = 1   # Estimator mode for RLS estimator - 0 for differential measurements, 1 for direct


# Kp,Ki,Kd,beta,gamma,MV_init,Lim_max,Lim_min,I_max,I_min,I_tog,PumpMax,PumpMin
BMS_Lim_max = 2500 # Maximum limit (power)
BMS_Lim_min = -2500# Minimum limit (power)
BMS_I_max   = 150  # Maximum limit (current)
BMS_I_min   = -150 # Minimum limit (current)
BMS_I_tog   = 50   # Pump current threshold / toggle
BMS_PumpMax = 2    # Flow in [m^3/h]
BMS_PumpMin = 0.0  # Flow in [m^3/h]

myBMS = CB.CuBMS(BMS_Lim_max, BMS_Lim_min, BMS_I_max, BMS_I_min, BMS_I_tog, BMS_PumpMax, BMS_PumpMin,BMS_EstMode)
#TempRegulator = CF.PID(0.01,0.05,0,0,0.1,0)
#BatteryModel = mbs.Sim_BMS()

#FlowRate = 0
#Cooling = 0

myBMS.EstimatorMode = 1
myBMS.RLSEstimator.Theta = np.array([1,-0.076,0.074]).reshape(3,1)

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
HOLDING_REGISTER_MAX        = HOLDING_REGISTER_START + HOLDING_REGISTERS ;


logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.ERROR)


# --------------------------------------------------------------------------- #
# Precalculations for battery and energy
# --------------------------------------------------------------------------- #
kilo = 1e3
mega = 1e6
giga = 1e9
milli = 1e-3
micro = 1e-6
nano = 1e-9
seconds_per_hour = 60*60
kWh_to_J = kilo * seconds_per_hour
J_to_kWh = 1/kWh_to_J
J_to_Wh = 1/seconds_per_hour

MIN_DISCHARGE_VOLTAGE = 5; #do not discharge below this voltage level.

TEST_POWER_REFERENCE = 2500; #Watt

CAP_LOSS = 1.0

#class Inverter(object):
#    def __init__(self, inverter_ip_address, modbus_id):
#        self.InverterIpAdress = inverter_ip_address
#        self.inverterModBusId = modbus_id
#    def get_grid_current(self):
#        return int(dummyvalue)
#    def get_input_voltage(self):
#        return int(dummyvalue)

INVERTER_STANDBY = 0
INVERTER_CHARGING = 1 #from grid to battery
INVERTER_DISCHARGING = -1 #from battery to grid

BATTERY_STANDBY = 0
BATTERY_CHARGING = 1 #from grid to battery
BATTERY_DISCHARGING = -1 #from battery to grid
BATTERY_UNDEFINED = 42 #from battery to grid

BMS_STANDBY = 0
BMS_CHARGING = 1
BMS_DISCHARGING = 2


class Sim_Inverter(object):
    """ simulate inverter """
    def __init__(self, inverter_ip_address, modbus_id):
        self.InverterIpAdress = inverter_ip_address
        self.inverterModBusId = modbus_id
        self.inverter_state = INVERTER_STANDBY
        self.DC_power = 0.0
        self.DC_voltage = 600
        self.DC_current = 0
        self.grid_power = 0.0
        self.grid_voltage = 380.0 #line to line RMS voltage 
        self.grid_current = 0.0
        self.efficiency = 0.93
        self.inverter_ambient_temp = 40.0;
        self.inverter_internal_temp = self.inverter_ambient_temp 

    def get_grid_current(self):
        self.grid_current = self.DC_power / self.grid_voltage
        return (self.grid_current)
    def get_grid_voltage(self):
        return (self.grid_voltage)
    def get_inverter_ambient_temp(self):
        return (self.inverter_ambient_temp)
    def get_inverter_internal_temp(self):
        if (self.inverter_state == INVERTER_CHARGING) or (self.inverter_state == INVERTER_DISCHARGING):
            self.inverter_internal_temp = self.inverter_ambient_temp + 20 ###@TODO add simple thermal model
        else:
            self.inverter_internal_temp = self.inverter_ambient_temp
        return (self.inverter_ambient_temp)
    def get_grid_power(self):
        self.grid_power = self.DC_power / self.efficiency
        return (self.grid_power)
    def get_DC_power(self):
        return (self.DC_power)
    def get_DC_voltage(self):
        return (self.DC_voltage)
    def get_DC_current(self):
        self.DC_current = self.DC_power/ self.DC_voltage
        return (self.DC_current)
    def get_inverter_state(self):
        return (self.inverter_state)
    def set_inverter_state(self, new_state):
        self.inverter_state = new_state
        return (self.inverter_state)
    def set_DC_power(self, power_reference):
        if float(power_reference) < 0.0:
            self.inverter_state = INVERTER_DISCHARGING
        self.DC_power = abs(float(power_reference))
        return(self.DC_power)

class Sim_pump(object):
    """simple pump simulation """
    def __init__(self):
        self.pump_speed = 0
        self.pump_running = False
    def get_pump_speed(self):
        if self.pump_running == True:
            self.pump_speed = 3000
        else:
            self.pump_speed = 0
        return(self.pump_speed)
    def start_pump(self):
        self.pump_running = True
        return(self.pump_running)
    def stop_pump(self):
        self.pump_running = False
        return(self.pump_running)
        
class Sim_battery(object):
    """simple stack thermal simulation """
    def __init__(self):
        self.P_max = 5000;
        self.P_loss_max = self.P_max * 0.1
        self.cell_voltage = 0.66 #V
        self.cell_number = 92.0 #-
        
        self.UMax = 72;
        self.UMin = 2;
        CB.setUMax = self.UMax
        CB.setUMin = self.UMin
        self.V_dc = self.cell_voltage*self.cell_number #rated voltage 
        self.E_tank = 50 * kWh_to_J #value in Joule from 25 kWh
        self.E_tank_kWh = self.E_tank/kWh_to_J
        self.V_charging = self.V_dc
        self.I_max = self.P_max / self.V_charging 
        
        self.R_1 = self.P_loss_max / (self.I_max*self.I_max)  #calculate resistance from losses and estimated current
        # C= Q/V_DC
        # t_full_charge ~= 4 * tidskonstant = 4* R*C ~=  E_tank/P_max
        # 4*R*C ~= E_tank/P_max <=> C ~=  E_tank/(4*R*P_max)
        self.C =  self.E_tank/(4*self.R_1*self.P_max) 
        self.time_const = self.R_1*self.C
        self.Q_tank = self.C* self.V_dc
        self.dt = 0.1 #delta t in seconds
        self.iternum = 100/self.dt
        CB.setTimeStep(myBMS, self.dt)
        
        self.chargingEnabled = False
        self.charging = True #if true we are charging, if false we are discharging (standby if chargingEnabled == false)
        self.battery_state = BATTERY_STANDBY
        CB.setChargeMode(myBMS, CB.BMS_IDLE)
        self.current_threshold = 0.5 #Amperes
        self.Charge_limit = 150 #A
        self.Discharge_limit = -150 #A
        self.Q_cap = self.Q_tank*0.5 #Coloumb
        self.V_cap = self.V_cap = self.Q_cap/self.C; #V
        self.V_stack = self.V_cap
        self.I_charge = 0 #A
        self.P_stack_input =0 #W
        self.charging_temp = 60
        self.standby_temp = 45
        self.stack_temperature = self.standby_temp;
        self.V_charging = 0
        self.SOC =0
        self.SOC_MAX = 0.95* self.E_tank #max charge to 95%
        self.SOC_MIN = 0.05* self.E_tank #max discharge to 5%
        
        self.FlowRate = 0 # Unit is L/s
        self.P_Cooling = 0 # Unit is W

        self.ThermalModel = TModel.RFB_ThermalModel(R_stack,R_pipes,R_he,R_ambient*2,C_stack,C_pipes,C_he,1)
        self.HydraulicModel = HModel.RFB_LumpedHydraulicModel(viscosity,permeability,density,length,width,depth,Nseries)

    def update_stack_simulation(self,Icharge):

        if (self.charging == True):
            self.V_charging =self.V_dc; #charging          
            if self.SOC >= self.SOC_MAX:
                self.chargingEnabled = False #no overcharging
        else:
            self.V_charging =0; #Discharging       
            #if self.SOC <= self.SOC_MIN:
            #    self.chargingEnabled = False #no undercharging
            #    print("MIN SOC reached")
            
        if self.chargingEnabled == False:
            self.V_charging = self.V_cap; #shut off charging)
            self.I_charge =0;
            self.stack_temperature = self.standby_temp ###@TODO better model
        else:
            self.ThermalModel.set_cooling(self.P_Cooling)
            self.stack_temperature = self.ThermalModel.Model_Timestep()
            self.HydraulicModel.Model_Timestep(self.FlowRate)
            
            self.I_charge = Icharge
            if self.I_charge > self.Charge_limit:
                self.I_charge = self.Charge_limit;
            if self.I_charge < self.Discharge_limit:
                self.I_charge = self.Discharge_limit;
                
            
                
        substep = 0
        steps_in_substep = 4
        subdt = self.dt / steps_in_substep # divide timestep
        while substep < steps_in_substep:
            substep = substep + 1
            self.Q_cap = self.Q_cap + subdt * self.I_charge;
            
            if self.Q_cap < 0:
                self.Q_cap = 0
                self.V_cap = 0            
                self.I_charge = 0
                print("Qcap < 0")
            else:
                self.Q_cap = self.Q_cap + subdt * self.I_charge;
                self.V_cap = self.Q_cap/self.C;
            
                
            self.V_stack =self.V_cap + self.I_charge * self.R_1;
            if self.V_stack < 0:
                self.V_stack = 0;
                print("Vstack < 0 with I_charge = " + str(self.I_charge))
                self.I_charge = (self.V_stack-self.V_cap)/self.R_1;
            self.P_stack_input = self.V_stack * self.I_charge;
            self.SOC = self.V_cap * self.Q_cap
            self.Q_cap = self.Q_cap * CAP_LOSS
            self.C = self.C * CAP_LOSS # Simulate capacity loss               
#        print('SIM_Battery UPD: Q = {}, V_cap={}'.format(self.Q_cap, self.V_cap))
        
    def get_stack_voltage(self):
        return (self.V_stack)
    def get_stack_current(self):
        return (self.I_charge)
    def get_stack_power (self):
        return (self.P_stack_input)
    def get_state_of_charge(self): #return J
        return (self.SOC)
    def get_stack_temperature(self):
        return(self.stack_temperature)
    def get_charge_limit(self):
        return (self.Charge_limit)

    def set_battery_state(self, new_state):
        if new_state == BATTERY_STANDBY:
            self.chargingEnabled = False
            self.charging = True
        elif new_state == BATTERY_CHARGING:
            self.chargingEnabled = True
            self.charging = True
        elif new_state == BATTERY_DISCHARGING:
            self.chargingEnabled = True
            self.charging = False
#        else:
#            self.chargingEnabled = False
#            self.charging = True
            
        self.battery_state = new_state
        return (self.battery_state)
    def get_battery_state(self):
        if (self.chargingEnabled == False):
            return_state = BATTERY_STANDBY
        elif (self.charging == True):
            return_state = BATTERY_CHARGING
        else:
            return_state = BATTERY_DISCHARGING
        return(return_state)
    def set_battery_dt(self, new_time_step):
        self.dt = new_time_step
        self.ThermalModel.set_dt(self.dt)
        return(self.dt)
    def get_battery_dt(self):
        return(self.dt)
    def get_flowrate(self):
        return(self.FlowRate)
    def set_flowrate(self,FlowRate):
        self.FlowRate = FlowRate
    def get_pressure(self):
        return(self.HydraulicModel.Get_StackPressure())
    def get_cooling(self):
        return(self.P_Cooling)
    def set_cooling(self,P_Cooling):
        self.P_Cooling = P_Cooling
        
        
class Sim_BMS(object):
    def __init__(self, ):
        self.power_reference =0.0 #W
        self.BMS_state = BMS_STANDBY
        self.battery = Sim_battery()
        self.battery.set_battery_state(BATTERY_STANDBY)
        
        self.pump_negative =  Sim_pump()
        self.pump_positive =  Sim_pump()

        self.inverter_obj = Sim_Inverter('192.0.128.2',0x1)
        self.pump_negative.stop_pump()
        self.pump_positive.stop_pump()
        self.inverter_obj.set_DC_power(0.0)
        self.inverter_obj.set_inverter_state (INVERTER_STANDBY)
        self.current_ref = 0
        self.simulationTime = 0;
        self.Rs =0;
        self.Rp =0;
        self.Cp =0;
        self.OCV =0;     
        
    def get_negative_pump_speed(self):
        return int(self.pump_negative.get_pump_speed())
    def get_positive_pump_speed(self):
        return int(self.pump_positive.get_pump_speed())
        
    def get_grid_current(self):
        return int(self.inverter_obj.get_grid_current())
    def get_input_voltage(self):
        return int(self.inverter_obj.get_grid_voltage())
    def get_inverter_ambient_temp(self):
        return int(self.inverter_obj.get_inverter_ambient_temp())
    def get_inverter_internal_temp(self):
        return int(self.inverter_obj.get_inverter_internal_temp())
    def get_grid_power(self):
        return int(self.inverter_obj.get_grid_power())
    def get_DC_power(self):
        return int(self.inverter_obj.get_DC_power())
    def get_DC_voltage(self):
        return int(self.inverter_obj.get_DC_voltage())
    def get_DC_current(self):
        return int(self.inverter_obj.get_DC_current())
    def get_inverter_state(self):
        return (self.inverter_obj.get_inverter_state())
    
    def get_battery_current(self):
        return int(self.battery.get_stack_current())
    def get_battery_voltage(self):
        return int(self.battery.get_stack_current())
    def get_battery_power(self):
        return int(self.battery.get_stack_power())
    def get_battery_soc(self):
        return int(self.battery.get_state_of_charge() * J_to_Wh + 0.5) #Convertfor modbus
    def set_power_reference(self, new_reference):
        self.power_reference = new_reference
        return int(self.power_reference)
    def get_power_reference(self):
        return int(self.power_reference)
    def start_charging(self):
        print("start charging")
        self.pump_negative.stop_pump()
        self.pump_positive.stop_pump()
        self.inverter_obj.set_DC_power(abs(self.power_reference))
        self.inverter_obj.set_inverter_state (INVERTER_CHARGING)
        self.battery.set_battery_state(BATTERY_CHARGING)
        #BMS_CHARGING_MODE = 1;
        #BMS_DISCHARGING_MODE = -1;
        #BMS_IDLE = 0;
        CB.setChargeMode(myBMS,CB.BMS_CHARGING_MODE)
        self.BMS_state = BMS_CHARGING
    def stop_charger(self):
        print("stop charger")
        self.pump_negative.stop_pump()
        self.pump_positive.stop_pump()
        self.inverter_obj.set_DC_power(0.0)
        self.inverter_obj.set_inverter_state (INVERTER_STANDBY)
        self.battery.set_battery_state(BATTERY_STANDBY)
        CB.setChargeMode(myBMS,CB.BMS_IDLE)
        self.BMS_state = BMS_STANDBY
    def start_discharging(self):
        print("start discharging")
        self.pump_negative.stop_pump()
        self.pump_positive.stop_pump()
        self.inverter_obj.set_DC_power(-abs(self.power_reference))
        self.inverter_obj.set_inverter_state (INVERTER_DISCHARGING)
        self.battery.set_battery_state(BATTERY_DISCHARGING)
        CB.setChargeMode(myBMS,CB.BMS_DISCHARGING_MODE)
        self.BMS_state = BMS_DISCHARGING
    def set_BMS_state(self, new_state):
        if new_state == BMS_STANDBY:
            self.stop_charger()
        elif new_state == BMS_CHARGING:
            self.start_charging()
        elif new_state == BMS_DISCHARGING:
            self.start_discharging()
        else:
            self.stop_charger()
        self.BMS_state = new_state
        return (self.BMS_state)
    def get_BMS_state(self):
        return (self.BMS_state)
        
#    def update_simulation_(self):
#        self.battery.update_stack_simulation()
#        self.inverter_obj.set_DC_power(self.battery.get_stack_power())
    def set_simulation_dt(self, new_time_step):
        self.battery.set_battery_dt(float(new_time_step))
        return int(new_time_step)
    
    def get_simulation_dt(self):
        return self.battery.get_battery_dt()
    
#    def update_simulation_cmj(self,Icharge):
#        self.battery.update_stack_simulation_CMJ(Icharge)
#        self.inverter_obj.set_DC_power(self.battery.get_stack_power())
    def get_simulation_time(self):
        return self.simulationTime
    
    def update_simulation(self):
        iters = 0
        while iters < self.battery.iternum:
            iters += 1
            self.simulationTime += self.battery.dt;
            
            BMS_state = self.get_BMS_state()
            if self.battery.SOC <= self.battery.SOC_MIN:
                if BMS_state == BMS_DISCHARGING:
                    self.stop_charger()
                    print("MIN SOC reached")
            elif self.battery.SOC >= self.battery.SOC_MAX:
                if BMS_state == BMS_CHARGING:
                    self.stop_charger()
                    print("MAX SOC reached")
    
            bat_voltage=self.battery.get_stack_voltage()*1.0
            #current_ref = 0
            power_ref = self.get_power_reference()*1.0
            #if bat_voltage > MIN_DISCHARGE_VOLTAGE:
            #DIVISION_FIX = 0.001;
            #print("Power ref = {}".format(power_ref) )
            if BMS_state == BMS_DISCHARGING:
                CB.setRefPower(myBMS,-power_ref)
            elif BMS_state == BMS_CHARGING:
                CB.setRefPower(myBMS,power_ref)
            else:
                CB.setRefPower(myBMS,0)
            
            #current_ref =  power_ref / (bat_voltage + DIVISION_FIX)
            current_ref_ctrl = CB.calcInvRef(myBMS)
            #print("Current ref {}".format(current_ref_ctrl))
    #        self.current_ref = 0.25 * self.current_ref + 0.75 * current_ref_ctrl
            self.current_ref = current_ref_ctrl
            if self.current_ref > self.battery.get_charge_limit():
                self.current_ref = self.battery.get_charge_limit()
            if BMS_state == BMS_STANDBY:
                self.current_ref = 0;
                #print("STANDBY: bat_voltage {} ".format(bat_voltage))     
            elif BMS_state == BMS_DISCHARGING:
                if bat_voltage > MIN_DISCHARGE_VOLTAGE:
                    #current_ref = -current_ref;
                    #                    print("DISCHARGING: bat_voltage {} at {} current ".format(bat_voltage , self.current_ref))     
                    self.current_ref = self.current_ref 
                else:
                    self.current_ref = self.current_ref 
                    #current_ref =0
#                    print("DISCHARGING low voltage: bat_voltage {} at {} current ".format(bat_voltage , self.current_ref))  
                    #print("DISCHARGING at low voltage");
            elif BMS_state == BMS_CHARGING:
                if bat_voltage <= MIN_DISCHARGE_VOLTAGE:
                    #current_ref = self.battery.get_charge_limit();
#                    print("CHARGING low voltage: bat_voltage {} at {} current ".format(bat_voltage , self.current_ref))  
                    self.current_ref = self.current_ref 
                    #print("CHARGING at low voltage")
#                else:
#                    print("CHARGING: bat_voltage {} at {} current_ref ".format(bat_voltage , self.current_ref))  
                    #print("Charging")
            else:
                #print(":UNKNOWN state - bat_voltage {} at {} current ".format(bat_voltage , self.current_ref))  
                print("UNKNOWN state??? -> standby")
                self.current_ref = 0;
                self.set_BMS_state(BMS_STANDBY)
            
            self.battery.update_stack_simulation(self.current_ref)
            self.inverter_obj.set_DC_power(self.battery.get_stack_power())
            CB.setMeasurements(myBMS, self.battery.get_stack_current(), 
                                      self.battery.get_stack_voltage(),
                                      self.battery.get_stack_power(),
                                      self.battery.get_pressure(),
                                      self.battery.get_state_of_charge());

            self.Rs, self.Rp, self.Cp, self.OCV, V_hat = myBMS.RLSEstimator.RLS_run(myBMS.U_meas,myBMS.I_meas)
#        print('Estimated parameters: Rs = {}, Rp = {}, Cp = {}'.format(Rs,Rp,Cp))
#        print('Estimated OCV = {}'.format(OCVhat))
#        print('True internal resistance = {}, true cap = {}'.format(self.battery.R_1,self.battery.C))
#        print('DC_current = {} , DC_power = {}'.format(self.get_DC_current(), self.battery.get_stack_power() ))
#        print('CB.soc = {} '.format(myBMS.SOC))

active_BMS = Sim_BMS()

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
	return (active_BMS.BMS_state);
def Read_SOC               (Value):
	return int15(active_BMS.get_battery_soc());
def Read_P_ref             (Value):
	return int15(active_BMS.get_power_reference());
def Read_GaussVolt         (Value):
	return (randvalue(0,100));            ###@TODO
def Read_SOC_Gauss         (Value):
	return (randvalue(0,100));            ###@TODO
def Read_BMS_Error         (Value):
	return (randvalue(0,1));            ###@TODO
def Read_Stack_Current     (Value):
	return int15(active_BMS.get_battery_current());
def Read_Stack_voltage     (Value):
	return int15(active_BMS.get_battery_voltage());
def Read_Chlorine          (Value):
	return (randvalue(0,30000));            ###@TODO
def Read_Temp_anolyte      (Value):
    return (randvalue(23,25));            ###@TODO
def Read_Temp_anolyte_In   (Value):
	return (randvalue(23,25));            ###@TODO
def Read_Flow_Anolyte      (Value):
	return (randvalue(3,50));            ###@TODO
def Read_Pressure_Anolyte  (Value):
    return (randvalue(0,5));            ###@TODO
def Read_Speed_Anolyte     (Value):
    return int15(active_BMS.get_positive_pump_speed());
def Read_Temp_catholyte    (Value):
    return (randvalue(23,25));            ###@TODO
def Read_Temp_catholyte_In (Value):
	return (randvalue(23,25));            ###@TODO
def Read_Flow_catholyte    (Value):
	return (randvalue(3,50));            ###@TODO
def Read_Pressure_catholyte(Value):
	return (randvalue(0,5));            ###@TODO
def Read_Speed_catholyte   (Value):
	return int15(active_BMS.get_positive_pump_speed());
def Read_DC_DC_ON          (Value):
	return int15(active_BMS.inverter_obj.get_inverter_state());
def Read_DC_DC_CURRENT     (Value):
	return int15(active_BMS.inverter_obj.get_DC_current());
def Read_DC_DC_VOLTAGE     (Value):
	return int15(active_BMS.inverter_obj.get_DC_voltage());
def Read_DC_DC_POWER       (Value):
	return int15(active_BMS.inverter_obj.get_DC_power());
def Read_DC_DC_TEMP        (Value):
	return int15(active_BMS.inverter_obj.get_inverter_internal_temp());
def Read_DC_DC_AMBIENT_TEMP(Value):
	return int15(active_BMS.inverter_obj.get_inverter_ambient_temp());
def Read_DC_DC_ERRORS      (Value):
	return (randvalue(0,1));            ###@TODO
def Read_AC_DC_ON          (Value):
	return int15(active_BMS.inverter_obj.get_inverter_state());
def Read_AC_DC_GRID_CURRENT(Value):
	return int15(active_BMS.inverter_obj.get_grid_current());
def Read_AC_DC_GRID_VOLTAGE(Value):
	return int15(active_BMS.inverter_obj.get_grid_voltage());
def Read_AC_DC_GRID_POWER  (Value):
	return int15(active_BMS.inverter_obj.get_grid_power());
def Read_DC_LINK_VOLTAGE   (Value):
	return int15(active_BMS.inverter_obj.get_DC_voltage());
def Read_AC_DC_TEMP        (Value):
	return int15(active_BMS.inverter_obj.get_inverter_internal_temp());
def Read_AC_DC_AMBIENT_TEMP(Value):
	return int15(active_BMS.inverter_obj.get_inverter_ambient_temp());
def Read_AC_DC_ERRORS      (Value):
	return (0);            ###@TODO
def Read_Speed_Thermal     (Value):
	return (randvalue(2800,2999));            ###@TODO
def Read_T_Thermal_tank    (Value):
	return (randvalue(23,25));            ###@TODO
def Read_Simulation_mode   (Value):
	return (Value);            ###@TODO
def Read_Time_contraction  (Value):
	return (Value);
def Read_Simulation_clock  (Value):
	return (Value);
def Read_thermal_temp_catholyte (value):
	return (randvalue(23,25));      
def Read_thermal_temp_anolyte   (value):
	return (randvalue(23,25));      
def read_anolyte_pressure_out   (value):
	return (randvalue(0,5));      
def read_catholyte_pressure_out (value):
	return (randvalue(0,5));      
def read_anolythe_temp_out      (value):
	return (randvalue(23,25));      
def read_catholyte_temp_out     (value):
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
    reg_anolyte_pressure_out    :read_anolyte_pressure_out  ,
    reg_catholyte_pressure_out  :read_catholyte_pressure_out,
    reg_anolythe_temp_out       :read_anolythe_temp_out     ,
    reg_catholyte_temp_out      :read_catholyte_temp_out    ,
    reg_rs_est                  :Read_Rs_est, 
    reg_rp_est                  :Read_Rp_est, 
    reg_cp_hi_est               :Read_Cp_hi_est,
    reg_cp_lo_est               :Read_CP_lo_est,
    reg_ocv_est                 :Read_OCV_est
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
            #print("Read {} at {} ".format(current_value , current_address))
        return super(CustomDataBlock, self).getValues(address, count)
    def setValues(self, address, value):
        """ Sets the requested values of the datastore

        :param address: The starting address
        :param values: The new values to be set
        """
        super(CustomDataBlock, self).setValues(address, value)

        # whatever you want to do with the written value is done here,
        # however make sure not to do too much work here or it will
        # block the server, espectially if the server is being written
        # to very quickly
        for idx in range(len(value)):
            current_address = (address+idx) - 1
            if current_address == Reg_Ctrl_BMS:
                active_BMS.set_BMS_state(value[idx])
                print("wrote {} at {} (BMS control)".format(value[idx], address+idx))                
            elif current_address ==  Reg_P_ref:
                active_BMS.set_power_reference(float (value[idx]))
                print("wrote {} at {} (p_ref)".format(value[idx], address+idx))
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

    interval = 0.33333

    server = ModbusTcpServer(context, identity=identity, address=('0.0.0.0', 5020))
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
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
