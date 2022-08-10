# -*- coding: utf-8 -*-
"""
Created on Thu May 20 17:58:52 2021

@author: Christian
"""

import ControlFuncs as CF
import numpy as np

BMS_CHARGING_MODE = 1;
BMS_DISCHARGING_MODE = -1;
BMS_IDLE = 0;


class CuBMS:
    def __init__(self,P_max,P_min,I_max,I_min,I_tog,PumpMax,PumpMin,EstMode,dt):
        self.FFController = CF.PID_Backcal(0.1,0.01,0,0,0,0)
        self.MinTempController = CF.PID_Backcal(0.1,0.1,0,1,0,0)
        self.MaxTempController = CF.PID_Backcal(0.1,0.1,0,1,0,0)
        self.StackTempController = CF.PID(33,0.1,0,100,-100,0)
        self.MinPressureController = CF.PID_Backcal(0.1,0.01,0,1,0,0)
        self.MaxPressureController = CF.PID_Backcal(0.1,0.01,0,1,0,0)
        self.InvRefPID = CF.PID_Backcal(0.01, 0.0001, 0, 1, 0, 0)
        mu = 1 # Estimator gain
        L = 0.99 # Estimator forgetting factor
        self.RLSEstimator = CF.CM_RLS(mu, L, 3, 100,EstMode)
        self.EstimatorMode = 1 # Set to 1 for 1-sample-hold measurement technique, 0 for RLS estimator
        self.U_max = P_max # Upper voltage limit (during charge)
        self.U_min = P_min # Lower voltage limit (during discharge)
        self.P_max = P_max # Power reference maximum
        self.P_min = P_min # Power reference minimum
        self.P_ref = 0     # Inverter power reference 
        self.I_max = I_max # Upper current limit
        self.I_min = I_min # Lower current limit
        self.I_tog = I_tog # Current toggle value for pump control
        self.InvRef = 0 # Inverter reference in Amperes
        
        self.PumpRef = 0 # Pump reference
        self.PumpMax = PumpMax # Upper pump reference limit
        self.PumpMin = PumpMin # Lower pump reference limit
        
        self.U_meas = 0 # Measured terminal voltage
        self.I_meas = 0 # Measured current input
        self.I_old = 0 # Measured current input at previous timestep
        self.P_meas = 0 # Measured power
        self.SOC = 1 # Measured/estimated state of charge
        
        self.dP_meas = 0 # Measured stack differential pressure
        self.dP_ub = 200 # Upper bound on stack differential pressure, millibars
        self.dP_lb = -200 # Lower bound on stack differential pressure
        
        self.T_meas = 25 # Measured stack temperature
        self.T_ub = 60 # Upper bound on stack temperature
        self.T_lb = 50 # Lower bound on stack temperature
        self.TempRef = 25 # Stack temperature reference
        self.TempHyst = 1 # Hysteresis variable for temperature control
        
        self.q_meas = 0 # Measured flowrate
        
        self.Mode = 0 # Charging mode. 0 = rest, -1 = discharge, 1 = charge
        self.t = 0 # Current time
        self.Hysteresis = 1 # Hysteresis toggle for max-power charge/discharge
        self.FF = 8 # Flow factor for optimal flow rate calculations
        self.FormC = 2 # Reactant concentration
        self.F = 96485
        self.N = 92
        self.R_est = 0
        self.U__old = 0 # OBS: only updated when Mode changes!
        self.C_est = 0
        self.Q_est = 0
        self.qual_thresh = 1
        self.U_tresh = 0.01 # Voltage quality threshold
        self.Auto = 1
        self.dt = dt # Timestep
        self.Vc = 0 # Capacitor voltage for internal calculations
        self.C_weight = 1/200
        self.R_weight = 1/10
        self.PumpMode = 1

    def calcInvRef(self):
        
        if self.Auto == 0:
            if self.Mode == 1: # Battery mode = charging
                if (self.U_meas < self.U_max) & (self.Hysteresis == 0):
                    self.InvRef = self.InvRefPID.run(self.t,self.U_max,self.U_meas,self.I_meas) # Calculate inverter reference with PID to update internal parameters
                    self.InvRef = self.I_max # Manually set current reference to max
                else:
                    self.InvRef = self.InvRefPID.run(self.t,self.U_max,self.U_meas,self.I_meas) # Calculate inverter reference
                    self.Hysteresis = 1
                if self.InvRef < 0:
                    self.InvRef = 0
            elif self.Mode == -1: # Battery mode = discharging
                if (self.U_meas > self.U_min) & (self.Hysteresis == 0):
                    self.InvRef = self.InvRefPID.run(self.t,self.U_min,self.U_meas,self.I_meas) # Calculate inverter reference with PID to update internal parameters
                    self.InvRef = self.I_min # Manually set current reference to minimum
                else:
                    self.InvRef = self.InvRefPID.run(self.t,self.U_min,self.U_meas,self.I_meas) # Calculate inverter reference
                    self.Hysteresis = 1
                if self.InvRef > 0:
                    self.InvRef = 0
            else: # Battery mode = rest
                self.InvRef = self.InvRefPID.run(self.t,0,self.U_meas,self.I_meas)
                self.InvRef = 0
        else:
            if self.Mode != 0:
                self.InvRef = self.InvRefPID.run(self.t,self.P_ref,self.P_meas,self.I_meas) # Calculate inverter reference
                if self.Mode == 1: # Battery mode = charging
        #            self.InvRef = self.InvRefPID.run(self.t,self.P_max,self.P_meas,self.I_meas) # Calculate inverter reference
                    if self.InvRef < 0:
                        self.InvRef = 0
                elif self.Mode == -1: # Battery mode = discharging
        #            self.InvRef = self.InvRefPID.run(self.t,self.P_min,self.P_meas,self.I_meas) # Calculate inverter reference
                    if self.InvRef > 0:
                        self.InvRef = 0
            else: # Battery mode = rest
                self.InvRef = self.InvRefPID.run(self.t,0,self.P_meas,self.I_meas)
                self.InvRef = 0
        return (self.InvRef)
    
    def calcPumpRef(self):
        
        if self.PumpMode == 0: # Bang-bang pump control based on charging current
            if np.abs(self.InvRef) < self.I_tog:
                self.PumpRef = self.PumpMin
            else:
                self.PumpRef = self.PumpMax
                
        if self.PumpMode == 1: # Nonlinear selector control, keeps pressure within bounds while preferring to deliver optimal flowrate.
            
                # if self.T_meas > self.TempRef + 5:
                #     self.TempHyst = 0
                # if abs(self.T_meas-self.TempRef) < 0.5:
                #     self.TempHyst = 1
                # if self.TempHyst == 0:    
            self.PumpRef = -self.StackTempController.run(self.t,self.TempRef,self.T_meas)
                    # self.PumpRef = min(self.PumpRef,100)
                    # self.PumpRef = max(self.PumpRef,25)
                # else:
                #     self.PumpRef = -self.StackTempController.run(self.t,self.TempRef,self.T_meas,self.PumpRef)
                #     self.PumpRef = min(self.PumpRef,100)
                #     self.PumpRef = max(self.PumpRef,0)
                    # self.PumpRef = 25
                    # self.PumpRef = max(self.PumpRef,0)
                    # self.PumpRef = min(self.PumpRef,100)
                
                #     # Feedforward = self.FF*(self.N*self.I_meas)/(self.F*(1-self.SOC+0.01)*self.FormC) # Feedforward part = optimal flow rate, charging
            #     # u_FF = self.FFController.run(self.t,Feedforward,self.q_meas,self.PumpRef)
            #     u_pmin = self.MinPressureController.run(self.t,self.dP_lb,self.dP_meas,self.PumpRef)
            #     u_pmax = self.MaxPressureController.run(self.t,self.dP_ub,self.dP_meas,self.PumpRef)
            #     u_tmin = -self.MinTempController.run(self.t,self.T_lb,self.T_meas,self.PumpRef)
            #     u_tmax = -self.MaxTempController.run(self.t,self.T_ub,self.T_meas,self.PumpRef)
            #     # u_pmin = max(u_pmin,0)
            #     # u_pmax = max(u_pmax,0)
            #     # MaxPart = max(u_pmin,u_tmin,0)
            #     # self.PumpRef = min(MaxPart,u_pmax,u_tmax,100)
            #     self.PumpRef = u_tmax
                    
            # elif self.Mode == -1 or self.Mode == 0: # Discharging
            #     # Feedforward = self.FF*(self.N*np.abs(self.I_meas))/(self.F*(self.SOC+0.01)*self.FormC) # Feedforward part = optimal flow rate, discharging
            #     # u_FF = self.FFController.run(self.t,Feedforward,self.q_meas,self.PumpRef)
            #     u_pmin = self.MinPressureController.run(self.t,self.dP_lb,self.dP_meas,self.PumpRef)
            #     u_pmax = self.MaxPressureController.run(self.t,self.dP_ub,self.dP_meas,self.PumpRef)
            #     u_tmin = -self.MinTempController.run(self.t,self.T_lb,self.T_meas,self.PumpRef)
            #     u_tmax = -self.MaxTempController.run(self.t,self.T_ub,self.T_meas,self.PumpRef)
            #     # u_min = max(u_min,0)
            #     # u_max = max(u_max,0)
            #     # MaxPart = max(u_pmin,u_tmin,0)
            #     # self.PumpRef = min(MaxPart,u_pmax,u_tmax,100)
            #     self.PumpRef = u_tmax
        
        if self.PumpRef > 100:
            self.PumpRef = 100
        elif self.PumpRef < 0:
            self.PumpRef = 0
        return self.PumpRef
        
    def DummyCurrentRef(self):
        Iref = self.P_ref/self.U_meas
        if Iref > self.I_max:
            Iref = self.I_max
        elif Iref < self.I_min:
            Iref = self.I_min
        return Iref
        
                
                
    def setChargeMode(self,Mode):
        if self.Mode != Mode:
            if self.EstimatorMode == 1:
                if np.abs(self.I_meas) > self.qual_thresh:
                    self.R_est = (1-self.R_weight)*self.R_est+self.R_weight*(self.U_meas-self.U_old)/self.I_meas
                else:
                    self.R_est = abs(self.RLSEstimator.Rs)
                    
        self.Mode = Mode
        
        if Mode == 1 or Mode == -1:
            self.Hysteresis = 0
        self.U_old = self.U_meas
            
    def getChargeMode(self):
        return self.Mode    
    
    def setPumpMode(self,PumpMode):
        self.PumpMode = PumpMode
    
    def getPumpMode(self):
        return self.PumpMode    
    
    def setTimeStep(self,dt):
        self.dt = dt
        
    def setCurrent(self,current):
        self.I_meas = current
            
    def setVoltage(self,voltage):
        self.U_meas = voltage
        
    def setPower(self,power):
        self.P_meas = power
        
    def setRefPower(self,RefPower):
        if self.Mode == 1:
            if RefPower > self.P_max:
                self.P_ref = self.P_max
            elif RefPower < 0:
                RefPower = 0
        if self.Mode == -1:
            if RefPower < self.P_min:
                self.P_ref = self.P_min
            elif RefPower > 0:
                RefPower = 0
                
        self.P_ref = RefPower
        
        
    def getRefPower(self):
        return self.P_ref
        
    def setPressure(self,dP):
        self.dP_meas = dP
        
    def getPressure(self):
        return self.dP_meas
    
    def setSOC(self,SOC):
        self.SOC = SOC
        
    def getSOC(self):
        return self.SOC
        
    def setFlow(self,q):
        self.q_meas = q
        
    def getFlow(self):
        return self.q_meas
        
    def setTemp(self,T):
        self.T_meas = T    
    
    def getTemp(self):
        return self.T_meas
    
    def setRefTemp(self,TempRef):
        self.TempRef = TempRef
        
    def getRefTemp(self):
        return self.TempRef
            
    def updateTime(self):
        self.t += self.dt
        
    def setTime(self,t):
        self.t = t
        
    def setUMax(self,u_max):
        self.U_max
        
    def setUMin(self,u_min):
        self.U_min
    
    def setMeasurements(self,current,voltage,power,dP,SOC,T,q):
        self.setCurrent(current)
        self.setVoltage(voltage)
        self.setPower(power)
        self.setPressure(dP)
        self.setSOC(SOC)
        self.setFlow(q)
        self.setTemp(T)
        self.updateTime()
        
    
    def calcCapacitance(self):
        self.Q_est += (self.I_meas + self.I_old)*self.dt/2 # Trapezoidal integrator
        self.Vc = self.U_meas - self.R_est*self.I_meas
        if self.Vc > self.U_tresh:
            self.C_est = self.C_weight*self.Q_est/self.Vc + (1-self.C_weight)*self.C_est
        self.I_old = self.I_meas
        return self.C_est

    
            

