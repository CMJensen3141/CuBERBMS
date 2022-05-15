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
        self.Vstack = 0.02 # Assume 20-liter stack volume  
        self.Vtanks = 1 # Assume tanks contain 1 m3 
        self.T_air = 22 # Ambient temperature
        self.T_a = self.T_air  # Anolyte temperature
        self.T_c = self.T_air  # Catholyte temperature
        self.T_s = self.T_air  # Stack temperature
        self.rho = 1.354*1000 # Electrolyte density in kg/m3
        self.Cp = 3.2*1000 # Specific heat capacity of electrolyte
        self.As = 6 # Surface area of each tank assuming cube shape
        self.U1 = (1/270.1 + 0.05/0.16 + 1/3.5)**-1
        self.U2 = (1/405.2 + 0.01/0.16 + 1/5.3)**-1
        self.UA = 4*self.U1*self.As + 2*self.U2*self.As
        self.Tsample = Tsample # Sample time
        
    def ModelTimestep(self,Qplus,Qminus,I):
        dStack = self.Cp*self.rho*(Qplus*(self.T_a-self.T_s)+Qminus*(self.T_c-self.T_s)+(I**2)*self.Rstack)
        self.T_s += dStack/(self.Cp*self.rho*self.Vstack)*self.Tsample
        
        dAno = Qplus*self.Cp*self.rho*(self.T_s-self.T_a)+self.UA*(self.T_air-self.T_a)
        self.T_a += dAno/(self.Cp*self.rho*self.Vtanks)*self.Tsample
        
        dCat = Qminus*self.Cp*self.rho*(self.T_s-self.T_c)+self.UA*(self.T_air-self.T_c)
        self.T_c += dCat/(self.Cp*self.rho*self.Vtanks)*self.Tsample
        
    def GetTemps(self):
        return self.T_s, self.T_a, self.T_c


#%% 

import numpy as np
import matplotlib.pyplot as plt
import RFB_FlowThermalModel as RFBFTM

fooMod = RFBFTM.RFB_FD_ThermalModel(1,1)

simlen = 1000

Tstack = np.empty([simlen,1])
Tano = np.empty([simlen,1])
Tcat = np.empty([simlen,1])

for ii in range(0,simlen):
    Tvec = fooMod.GetTemps()
    Tstack[ii] = Tvec[0]
    Tano[ii] = Tvec[1]
    Tcat[ii] = Tvec[2]
    if ii < simlen/2:
        fooMod.ModelTimestep(0.0001,0.001,1)
    else:
        fooMod.ModelTimestep(0.0001,0.0001,0)
        
        
plt.figure()
plt.plot(Tstack,'b--')
plt.plot(Tano,'r--')
plt.plot(Tcat,'g--')
plt.legend(["Stack","Anolyte", "Catholyte"])
    
    
