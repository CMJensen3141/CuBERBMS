# -*- coding: utf-8 -*-
"""
Created on Fri Jul 23 19:23:05 2021
This module implements an equivalent circuit-based thermal model of a redox flow battery, using a three-layer Cauer network.


@author: Christian Møller Jensen, Bac. Eng. EE., Aarhus University.

Acknowledgements: The model in this module is based on work by Xiong et al., see doi.org/10.1109/ACCESS.2019.2952212.
Parameter values from Xiong et al are provided in comments of init function.     
"""

import numpy as np

class RFB_ThermalModel:
    def __init__(self,R_stack,R_pipes,R_he,R_ambient,C_stack,C_pipes,C_he,dt):
        self.R_stack = R_stack     # 2.1*10^-4
        self.R_pipes = R_pipes     # 1.0*10^-3
        self.R_he = R_he           # 3.8*10^-3
        self.R_ambient = R_ambient # 8.4*10^-3 
        self.C_stack = C_stack     # 4.7*10^3 
        self.C_pipes = C_pipes     # 5.2*10^4
        self.C_he = C_he           # 4.7*10^5 
        self.T_ambient = 25        # 25.2   
        self.P_disch = 402*2         # 402.048 for discharge. This should be amended to consider flow rate in the future.
        self.P_charge = 267*2        # 267.308 for charge. This should be amended to consider flow rate in the future.
        self.P_mode = 1            # 1 for charge, -1 for discharge
        self.P_cooling = 0         # Heat removal via forced cooling at the heat exchanger 
        self.T_stack = self.T_ambient # Initialise all temperatures at ambient value
        self.T_pipes = self.T_ambient
        self.T_he = self.T_ambient
        self.dt = dt               # Size of model timestep
    
        
    def Model_Timestep(self):
        if self.P_mode == -1:
            P_total = self.P_disch
        else:
            P_total = self.P_charge
        
        self.T_stack += (-1/self.R_pipes*self.T_stack+1/self.R_pipes*self.T_pipes+P_total)/(self.C_stack)*self.dt
        
        self.T_pipes += (1/self.R_pipes*self.T_stack-(1/self.R_he+1/self.R_pipes)*self.T_pipes+1/self.R_he*self.T_he)/(self.C_pipes)*self.dt
        
        self.T_he += (1/self.R_he*self.T_pipes-(1/self.R_he+1/self.R_ambient)*self.T_he+1/self.R_ambient*self.T_ambient+self.P_cooling)/(self.C_he)*self.dt
        
    def set_dt(self,dt):
        self.dt = dt
        
    def set_resistances(self,args):
        if len(args) < 1:
            raise ValueError("Too few resistances specified!")
        if len(args) >= 1:    
            self.R_stack = args[0]
        if len(args) >=2:
            self.R_pipes = args[1]
        if len(args) >=3:    
            self.R_he = args[2]
        if len(args) >=4:    
            self.R_ambient = args[3]
        if len(args) >=5:
            raise ValueError("Too many resistances specified!")
            
    def set_capacitors(self,args):
        if len(args) < 1:
            raise ValueError("Too few capacitances specified!")
        if len(args) >= 1:    
            self.C_stack = args[0]
        if len(args) >=2:
            self.C_pipes = args[1]
        if len(args) >=3:    
            self.C_he = args[2]
        if len(args) >=4:    
            raise ValueError("Too many capacitances specified!")
    
    def set_power(self,P_disch,P_charge):
        self.P_disch = P_disch
        self.P_charge = P_charge
        
    def set_cooling(self,P_cooling):
        self.P_cooling = P_cooling
   
    def set_ambient_temp(self,Temp):
        self.T_ambient = Temp
        
    def set_mode(self,mode):
        self.P_mode = mode
   
    def get_resistances(self):
        return [self.R_stack,self.R_pipes,self.R_he,self.R_ambient]
       
    
    def get_capacitors(self):
        return [self.C_stack,self.C_pipes,self.C_he]

    def get_dt(self):
        return self.dt
    
    def get_power(self):
        if self.P_mode == 1:
            return self.P_charge
        if self.P_mode == -1:
            return self.P_disch
    
    def get_system_temps(self):
        return [self.T_stack,self.T_pipes,self.T_he]
        
    def get_ambient_temp(self):
        return self.T_ambient
    
    def get_mode(self):
        return self.P_mode
    
    def get_cooling(self):
        return self.P_cooling

        
#%% "Unit test" environment

import matplotlib.pyplot as plt
import ControlFuncs as CF

Regulator = CF.PID(1,0.1,0,0,0.1,0)

SP = 50

R_stack = 2.1e-04
R_pipes = 1.0e-03
R_he = 3.8e-03
R_ambient = 2*8.4e-03
C_stack = 4.7e03
C_pipes = 5.2e04
C_he = 4.7e05

TestMod = RFB_ThermalModel(R_stack,R_pipes,R_he,R_ambient*2,C_stack,C_pipes,C_he,1)

numCycles = 100


cycleLen = int(50*60/TestMod.dt)
count = 0
T_stack = np.empty(numCycles*cycleLen)
T_pipes = np.empty(numCycles*cycleLen)
T_he = np.empty(numCycles*cycleLen)
P_total = np.empty(numCycles*cycleLen)
P_cool = np.empty(numCycles*cycleLen)


for ii in range(0,numCycles*cycleLen):
    CoolP = Regulator.run(ii,SP,TestMod.get_system_temps()[0],TestMod.get_cooling())
    TestMod.set_cooling(CoolP)
    TestMod.Model_Timestep()
    TempVec = TestMod.get_system_temps()
    T_stack[ii] = TempVec[0]
    T_pipes[ii] = TempVec[1]
    T_he[ii] = TempVec[2]
    P_total[ii] = TestMod.get_power()
    P_cool[ii] = TestMod.get_cooling()
    count += 1
    mode = TestMod.get_mode()
    if count == cycleLen:
        TestMod.set_mode(-mode)
        count = 0
        
    
plt.figure()
plt.plot(T_stack,'b--')
plt.plot(T_pipes,'r--')
plt.plot(T_he,'g--')
plt.legend(["Stack","Pipes", "Heat Exchanger"])

plt.figure()
plt.plot(P_total,'b')
plt.plot(P_cool,'r')
plt.plot(P_total-P_cool,'--k')
plt.legend(["Heat input to system","Heat removed at heat exchanger","Total heat"])