# -*- coding: utf-8 -*-
"""
Created on Tue May 10 21:57:41 2022

@author: Christian Møller Jensen, Bac. Eng. EE., Aarhus University.

Implements a flow-dependent thermal model of a classic single-stack, double-tank RFB. All units are SI.

Acknowledgements: The model in this module is based on work by Kazacos et al., see doi.org/10.1016/J.JPOWSOUR.2011.11.079 
"""

import numpy as np

class RFB_FD_ThermalModel: 
    def __init__(self,Rstack,Tsample):
        self.Rstack = Rstack
        self.Vstack = 0.02 # Stack volume in m^3
        self.Vtanks = 1 # Tank volumes (assumed equal) in m^3
        self.T_air = 40 # Ambient temperature
        self.T_a = self.T_air  # Anolyte temperature
        self.T_c = self.T_air  # Catholyte temperature
        self.T_s = self.T_air  # Stack temperature
        # self.rho = 1.354 # Electrolyte density in g/cm3
        self.rho = 1354000 # Electrolyte density in g/m^3
        self.Cp = 3.2 # Specific heat capacity of electrolyte, J/(g*K)^-1
        self.As = 6 # Surface area of each tank assuming cube shape
        self.U1 = (1/270.1 + 0.05/0.16 + 1/3.5)**-1
        self.U2 = (1/405.2 + 0.01/0.16 + 1/5.3)**-1
        self.UA = 4*self.U1*self.As + 2*self.U2*self.As
        self.UA = 10*self.As # Rough number under assumption of cubic tanks and unforced convection
        self.Tsample = Tsample # Sample time
        
    def ModelTimestep(self,Qplus,Qminus,I):
        dStack = self.Cp*self.rho*(Qplus*(self.T_a-self.T_s)+Qminus*(self.T_c-self.T_s))+(I**2)*self.Rstack
        self.T_s += dStack/(self.Cp*self.rho*self.Vstack)*self.Tsample
        
        dAno = Qplus*self.Cp*self.rho*(self.T_s-self.T_a)+self.UA*(self.T_air-self.T_a)
        self.T_a += dAno/(self.Cp*self.rho*self.Vtanks)*self.Tsample
        
        dCat = Qminus*self.Cp*self.rho*(self.T_s-self.T_c)+self.UA*(self.T_air-self.T_c)
        self.T_c += dCat/(self.Cp*self.rho*self.Vtanks)*self.Tsample
        
    def GetTemps(self):
        return self.T_s, self.T_a, self.T_c


3#%% 

import numpy as np
import matplotlib.pyplot as plt
import RFB_FlowThermalModel as RFBFTM

literstocubic = 1.66666667*10**-5
Ts = 1


fooMod = RFBFTM.RFB_FD_ThermalModel(0.073,Ts)

simtime = 48 # Simulation time in hours

simlen = round((simtime*3600)/Ts) 

Tstack = np.empty([simlen,1])
Tano = np.empty([simlen,1])
Tcat = np.empty([simlen,1])

flow1 = 1*literstocubic
flow2 = 1*literstocubic

for ii in range(0,simlen):
    Tvec = fooMod.GetTemps()
    Tstack[ii] = Tvec[0]
    Tano[ii] = Tvec[1]
    Tcat[ii] = Tvec[2]
    if ii < simlen/2:
        fooMod.ModelTimestep(flow1,flow1,100)
    else:
        fooMod.ModelTimestep(flow2,flow2,-100)
        
taxis = np.linspace(0,simtime,round(simlen/Ts))
        
plt.figure()
plt.plot(taxis,Tstack,'b--')
plt.plot(taxis,Tano,'r--')
plt.plot(taxis,Tcat,'g--')
plt.legend(["Stack","Anolyte", "Catholyte"])
    
    
