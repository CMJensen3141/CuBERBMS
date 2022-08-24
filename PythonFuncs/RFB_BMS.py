# -*- coding: utf-8 -*-
"""
Created on Thu Aug 18 13:30:22 2022

@author: chris
"""

import random
import CuBMS as CB
import numpy as np
import ControlFuncs as CF
import RFB_CombinedModel as ComMod
from bms_registers import * 
from UnitConversions import *
import modbus_class as mc
import time

# --------------------------------------------------------------------------- #
# set invariant parameters of imported models
# --------------------------------------------------------------------------- #

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

AtmPres = 1000 # Atmospheric pressure in mbar

TimeStep = 100 # Step size in simulation is 1 second
Speed_Factor = 60 # Simulation speed-up factor

MIN_DISCHARGE_VOLTAGE = 5; #do not discharge below this voltage level.

TEST_POWER_REFERENCE = 2500; #Watt

CAP_LOSS = 1.0

myBMS = CB.CuBMS(BMS_Lim_max, BMS_Lim_min, BMS_I_max, BMS_I_min, BMS_I_tog, BMS_PumpMax, BMS_PumpMin,BMS_EstMode,TimeStep)
#TempRegulator = CF.PID(0.01,0.05,0,0,0.1,0)
# BatteryModel = Sim_BMS()

#FlowRate = 0
#Cooling = 0

myBMS.EstimatorMode = 1
myBMS.RLSEstimator.Theta = np.array([1,-0.076,0.074]).reshape(3,1)

class Sim_Inverter(object):
    """ simulate inverter """
    def __init__(self):
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
        self.SimStatus = True
        self.converter = mc.Client()  
        self.converterclient = 0
        self.FirstRun = True
        self.TimeStamp = -1

    def OpenInverterConnection(self):
        [cc,connected] = self.converter.connect_to_client()
        self.converterclient = cc
        return connected
    
    def CloseInverterConnection(self):
        self.converterclient.disconnect_from_client()
        
    def InverterShutdown(self):
        self.converter.stop_transmission()
        self.CloseInverterConnection()
        
    def SetPowerRef(self,ref):
        ref = abs(ref)
        if self.inverter_state == INVERTER_CHARGING:
            if self.FirstRun == True:
                self.TimeStamp = time.time()
                self.converter.charge_battery()
                self.converter.start_transmission(ref)
                self.FirstRun = False
            else:
                if (self.TimeStamp + 60) > time.time():
                    self.converter.set_AC_power(ref)
        elif self.inverter_state == INVERTER_DISCHARGING:
            if self.FirstRun == True:
                self.TimeStamp = time.time()
                self.converter.discharge_battery()
                self.converter.start_transmission(ref)
                self.FirstRun = False
            else:
                if (self.TimeStamp + 60) > time.time():
                    self.converter.set_AC_power(ref)
        else:
            print("Attempting to change power reference while in standby mode.")            
        
    
    def set_sim_status(self,status: bool):
        if status == False or status == True:
            self.SimStatus = status
            if status == False:
                self.OpenInverterConnection()
        else:
            self.SimStatus = True
            print("I only accept boolean variables: set_sim_status")

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
        self.max_speed = 3000
    def get_pump_speed(self):
        return float(self.pump_speed)
    def get_pump_duty(self):
        return float((self.get_pump_speed()/self.max_speed)*100)
    def set_pump_speed(self,speed):
        if speed < 0:
            speed = 0
        if speed > self.max_speed:
            speed = self.max_speed
        self.pump_speed = speed
    def set_pump_duty(self,duty):
        if duty < 0.0:
            duty = 0.0
        if duty > 100.0:
            duty = 100.0
        self.pump_speed = (self.max_speed*duty)/100.0
    def start_pump(self):
        self.pump_running = True
        self.set_pump_speed(self.max_speed)
        return(self.pump_running)
    def stop_pump(self):
        self.pump_running = False
        self.set_pump_speed(0)
        return(self.pump_running)
        
class Sim_battery(object):
    def __init__(self):
        self.P_max = 5000;
        self.P_loss_max = self.P_max * 0.1
        self.cell_voltage = 0.66 #V
        self.cell_number = 92.0 #-
        
        self.UMax = 72;
        self.UMin = 2;
        myBMS.setUMax = self.UMax
        myBMS.setUMin = self.UMin
        self.V_dc = self.cell_voltage*self.cell_number #rated voltage 
        self.E_tank = 50 * kWh_to_J #value in Joule from 25 kWh
        self.E_tank_kWh = self.E_tank/kWh_to_J
        self.I_max = self.P_max / self.V_dc
        
        self.R_1 = self.P_loss_max / (self.I_max*self.I_max)  #calculate resistance from losses and estimated current
        self.C =  self.E_tank/(4*self.R_1*self.P_max) 
        self.time_const = self.R_1*self.C
        self.Q_tank = self.C* self.V_dc
        self.dt = TimeStep #delta t in seconds
        self.SpeedUp = Speed_Factor
        myBMS.setTimeStep(self.dt)
        
        self.chargingEnabled = False
        self.charging = True #if true we are charging, if false we are discharging (standby if chargingEnabled == false)
        self.battery_state = BATTERY_STANDBY
        myBMS.setChargeMode(CB.BMS_IDLE)
        self.current_threshold = 0.5 #Amperes
        self.Charge_limit = 150 #A
        self.Discharge_limit = -150 #A
        self.Q_cap = self.Q_tank*0.5 #Coloumb
        self.V_cap = self.V_cap = self.Q_cap/self.C; #V
        self.V_stack = self.V_cap
        self.Icharge = 0 #A
        self.P_stack_input =0 #W
        self.charging_temp = 60
        self.standby_temp = 45
        self.stack_temperature = self.standby_temp;
        self.V_charging = 0
        self.SOC =0
        self.SOC_MAX = 0.95
        self.SOC_MIN = 0.05
        
        self.FlowRate = 0 # Unit is L/s
        self.P_Cooling = 0 # Unit is W
        
        self.c_total = 2500 # Total concentration in mol/m^3 of electrolyte species
        self.T_ambient = 30 # Ambient temperature in degrees celsius
        
        self.Inverter = Sim_Inverter()
        self.CombinedModel = ComMod.RFB_CombinedModel(self.dt, self.R_1, self.T_ambient, self.c_total)
        
        self.SimStatus = True # Indicate whether we are simulating the battery dynamics

    def update_stack_simulation(self,w_ano,w_cat,Icharge):

        if (self.charging == True):
            if self.CombinedModel.GetSOC() >= self.SOC_MAX:
                self.chargingEnabled = False #no overcharging
                Icharge = 0
        else:     
            if self.CombinedModel.GetSOC() <= self.SOC_MIN:
                self.chargingEnabled = False #no undercharging
                Icharge = 0
                print("I think the current SOC is " + str(self.CombinedModel.GetSOC()))
            
        self.CombinedModel.Model_Timestep(w_ano,w_cat,Icharge)
        self.V_stack = self.CombinedModel.GetVoltage(); 
        self.set_stack_current(Icharge)
        self.set_state_of_charge(self.CombinedModel.GetSOC())

        
    def get_stack_voltage(self):
        if self.Inverter.SimStatus == True:
            return (self.V_stack)
        else:
            voltage = self.Inverter.converter.get_battery_voltage()
            return voltage
    def get_stack_current(self):
        if self.Inverter.SimStatus == True:
            return (self.Icharge)
        else:
            current = self.Inverter.converter.get_battery_current()
            return current
    def set_stack_current(self,current):
        self.Icharge = current
    def get_stack_power (self):
        if self.Inverter.SimStatus == True:
            return (self.P_stack_input)
        else:
            [perphase,power] = self.Inverter.converter.get_active_power()
            return power
    def get_state_of_charge(self): #return J
        return (self.SOC)
    def get_stack_temperature(self):
        if self.SimStatus == True:
            return(self.stack_temperature)
    def get_charge_limit(self):
        return (self.Charge_limit)
    
    def set_state_of_charge(self,soc):
        self.SOC = soc
    
    def get_speedup_factor(self):
        return self.SpeedUp
    
    def set_speedup_factor(self,speedup):
        self.SpeedUp = speedup

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
        return self.CombinedModel.GetAnoPressure_Stack()
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

        self.pump_negative.stop_pump()
        self.pump_positive.stop_pump()
        self.battery.Inverter.set_DC_power(0.0)
        self.battery.Inverter.set_inverter_state (INVERTER_STANDBY)
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
        return int(self.battery.Inverter.get_grid_current())
    def get_input_voltage(self):
        return int(self.battery.Inverter.get_grid_voltage())
    def get_inverter_ambient_temp(self):
        return int(self.battery.Inverter.get_inverter_ambient_temp())
    def get_inverter_internal_temp(self):
        return int(self.battery.Inverter.get_inverter_internal_temp())
    def get_grid_power(self):
        return int(self.battery.Inverter.get_grid_power())
    def get_DC_power(self):
        return int(self.battery.Inverter.get_DC_power())
    def get_DC_voltage(self):
        return int(self.battery.Inverter.get_DC_voltage())
    def get_DC_current(self):
        return int(self.battery.Inverter.get_DC_current())
    def get_inverter_state(self):
        return (self.battery.Inverter.get_inverter_state())
    
    def get_battery_current(self):
        return int(self.battery.get_stack_current())
    def get_battery_voltage(self):
        return int(self.battery.get_stack_voltage())
    def get_battery_power(self):
        return int(self.battery.get_stack_power())
    def get_battery_soc(self):
        return self.CombinedModel.GetSOC()
    def set_power_reference(self, new_reference):
        self.power_reference = new_reference
        return int(self.power_reference)
    def get_power_reference(self):
        return int(self.power_reference)
    def start_charging(self):
        print("start charging")
        self.pump_negative.set_pump_duty(100)
        self.pump_positive.set_pump_duty(100)
        self.battery.Inverter.set_DC_power(abs(self.power_reference))
        self.battery.Inverter.set_inverter_state (INVERTER_CHARGING)
        self.battery.set_battery_state(BATTERY_CHARGING)
        myBMS.setChargeMode(CB.BMS_CHARGING_MODE)
        self.BMS_state = BMS_CHARGING
    def stop_charger(self):
        print("stop charger")
        self.pump_negative.stop_pump()
        self.pump_positive.stop_pump()
        self.battery.Inverter.set_DC_power(0.0)
        self.battery.Inverter.set_inverter_state (INVERTER_STANDBY)
        self.battery.set_battery_state(BATTERY_STANDBY)
        myBMS.setChargeMode(CB.BMS_IDLE)
        self.BMS_state = BMS_STANDBY
    def start_discharging(self):
        print("start discharging")
        self.pump_negative.set_pump_duty(100)
        self.pump_positive.set_pump_duty(100)
        self.battery.Inverter.set_DC_power(-abs(self.power_reference))
        self.battery.Inverter.set_inverter_state (INVERTER_DISCHARGING)
        self.battery.set_battery_state(BATTERY_DISCHARGING)
        myBMS.setChargeMode(CB.BMS_DISCHARGING_MODE)
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
        
    def set_simulation_dt(self, new_time_step):
        self.battery.set_battery_dt(float(new_time_step))
        return int(new_time_step)
    
    def get_simulation_dt(self):
        return self.battery.get_battery_dt()
    
    def get_simulation_time(self):
        return self.simulationTime
    
    def update_simulation(self):
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
        power_ref = self.get_power_reference()*1.0
        
        if BMS_state == BMS_DISCHARGING:
            myBMS.setRefPower(-power_ref)
        elif BMS_state == BMS_CHARGING:
            myBMS.setRefPower(power_ref)
        else:
            myBMS.setRefPower(0)
        
        
        if self.battery.Inverter.SimStatus == True: 
            self.current_ref = myBMS.DummyCurrentRef()
            if self.current_ref > self.battery.get_charge_limit():
                self.current_ref = self.battery.get_charge_limit()
            if BMS_state == BMS_STANDBY:
                self.current_ref = 0;  
            elif BMS_state == BMS_DISCHARGING:
                if bat_voltage > MIN_DISCHARGE_VOLTAGE:  
                    self.current_ref = self.current_ref 
                else:
                    self.current_ref = self.current_ref 
            elif BMS_state == BMS_CHARGING:
                if bat_voltage <= MIN_DISCHARGE_VOLTAGE: 
                    self.current_ref = self.current_ref 
            else:
                print("UNKNOWN state??? -> standby")
                self.current_ref = 0;
                self.set_BMS_state(BMS_STANDBY)
        elif self.battery.SimStatus == True:
            self.current_ref = myBMS.DummyCurrentRef() # Don't need to do a bunch of fancy stuff as hopefully the inverter is handling that
        else:
            self.current_ref = self.battery.get_stack_current()
        
        self.battery.update_stack_simulation(self.pump_positive.get_pump_duty(),
                                             self.pump_negative.get_pump_duty(),
                                             self.current_ref);
        self.battery.Inverter.set_DC_power(self.battery.get_stack_power())
        myBMS.setMeasurements(self.battery.get_stack_current(), 
                                  self.battery.get_stack_voltage(),
                                  self.battery.get_stack_power(),
                                  self.battery.get_pressure(),
                                  self.battery.get_state_of_charge(),
                                  self.battery.get_stack_temperature(),
                                  self.battery.get_flowrate());
        
        # print("Positive pump: " + str(self.pump_positive.get_pump_duty()))
        # print("Negative pump: " +str(self.pump_negative.get_pump_duty()))
        # print("Current reference:" + str(self.current_ref))
        
if __name__ == "__main__":
    Test_Trumpf()