# -*- coding: utf-8 -*-
"""
Created on Tue May 10 21:57:41 2022

@author: Christian Møller Jensen, Bac. Eng. EE., Aarhus University.

Implements a flow-dependent thermal model of a classic single-stack, double-tank RFB. All units are SI.

Acknowledgements: The model in this module is based on work by Kazacos et al., see doi.org/10.1016/J.JPOWSOUR.2011.11.079 
"""

import numpy as np
import math
from numba import njit, jit

class RFB_FD_ThermalModel: 
    def __init__(self,Rstack,T_ambient,Tsample_extern,Tsample_intern):
        self.Rstack = Rstack
        self.Vstack = 0.01 # Stack volume in m^3
        self.Vtanks = 0.05 # Tank volumes (assumed equal) in m^3
        self.T_air = T_ambient # Ambient temperature
        # self.T_a = self.T_air  # Anolyte temperature
        # self.T_c = self.T_air  # Catholyte temperature
        # self.T_s = self.T_air  # Stack temperature
        self.T_a = 50  # Anolyte temperature
        self.T_c = 50  # Catholyte temperature
        self.T_s = 50  # Stack temperature
        # self.rho = 1.354 # Electrolyte density in g/cm3 
        self.rho = 1310000 # Electrolyte density in g/m^3
        self.Cp = 3.2 # Specific heat capacity of electrolyte, J/(g*K)^-1
        self.As = 6 # Surface area of each tank assuming cube shape
        self.U1 = (1/270.1 + 0.05/0.16 + 1/3.5)**-1
        self.U2 = (1/405.2 + 0.01/0.16 + 1/5.3)**-1
        self.UA = 4*self.U1*self.As + 2*self.U2*self.As
        self.UA = 10*self.As # Rough number under assumption of cubic tanks and unforced convection
        self.Tsample_extern = Tsample_extern # Sample time of external simulation
        self.Tsample_intern = Tsample_intern # Internal sample time to avoid stiffness issues
        self.NumbaStep = self.NumbaGenerator() # Generate the efficient compiled model timestepper
        
    def ModelTimestep(self,Qplus,Qminus,I):
        dStack = self.Cp*self.rho*(Qplus*(self.T_a-self.T_s)+Qminus*(self.T_c-self.T_s))+(I**2)*self.Rstack
        self.T_s += dStack/(self.Cp*self.rho*self.Vstack)*self.Tsample
        
        dAno = Qplus*self.Cp*self.rho*(self.T_s-self.T_a)+self.UA*(self.T_air-self.T_a)
        self.T_a += dAno/(self.Cp*self.rho*self.Vtanks)*self.Tsample
        
        dCat = Qminus*self.Cp*self.rho*(self.T_s-self.T_c)+self.UA*(self.T_air-self.T_c)
        self.T_c += dCat/(self.Cp*self.rho*self.Vtanks)*self.Tsample
        
    def NumbaGenerator(self):
        Cp = self.Cp; rho = self.rho; Rstack = self.Rstack; Vstack = self.Vstack; Vtanks = self.Vtanks; UA = self.UA; Ts_ext = self.Tsample_extern; Ts_int = self.Tsample_intern; # Local copies of necessary static variables
        
        if math.ceil(Ts_ext/Ts_int) > 1:
            numsteps = math.ceil(Ts_ext/Ts_int)
        else:
            numsteps = 1
        
        @njit
        def NumbaTimestep(T_s,T_a,T_c,T_air,Qplus,Qminus,I):
                for ii in range(0,numsteps):
                    dStack = Cp*rho*(Qplus*(T_a-T_s)+Qminus*(T_c-T_s))+(I**2)*Rstack
                    T_s += dStack/(Cp*rho*Vstack)*Ts_int
                    
                    # dAno = Qplus*Cp*rho*(T_s-T_a)+UA*(T_air-T_a)
                    # T_a += dAno/(Cp*rho*Vtanks)*Ts_int
                    
                    # dCat = Qminus*Cp*rho*(T_s-T_c)+UA*(T_air-T_c)
                    # T_c += dCat/(Cp*rho*Vtanks)*Ts_int
                
                    # Assuming tanks are kept at constant temperature for now
                    T_a = T_a; T_c = T_c;
                Tvec = [T_s,T_a,T_c]
                return Tvec
                    
        return NumbaTimestep
        
    def GetTemps(self):
        return self.T_s, self.T_a, self.T_c
    
    def SetTemps(self,T_s,T_a,T_c):
        self.T_s = T_s
        self.T_a = T_a
        self.T_c = T_c



if __name__ == "__main__": # Unit tests, run only if executing this file as main window

    import numpy as np
    import matplotlib.pyplot as plt
    
    literstocubic = 1.66666667*10**-5
    Ts = 10
    
    
    fooMod = RFB_FD_ThermalModel(0.073, 25, Ts,0.01)
    
    simtime = 40 # Simulation time in hours
    
    simlen = round((simtime*3600)/Ts) 
    
    Tstack = np.empty([simlen,1])
    Tano = np.empty([simlen,1])
    Tcat = np.empty([simlen,1])
    
    flow1 = 10*literstocubic
    flow2 = 10*literstocubic
    
    for ii in range(0,simlen):
        Tvec = fooMod.GetTemps()
        Tstack[ii] = Tvec[0]
        Tano[ii] = Tvec[1]
        Tcat[ii] = Tvec[2]
        if ii < simlen/2:
            Tout = fooMod.NumbaStep(Tvec[0],Tvec[1],Tvec[2],fooMod.T_air,flow1,flow2,150)
        else:
            #fooMod.ModelTimestep(flow2,flow2,-150)
            Tout = fooMod.NumbaStep(Tvec[0],Tvec[1],Tvec[2],fooMod.T_air,flow1,flow2,-150)
        fooMod.SetTemps(Tout[0], Tout[1], Tout[2])
        
        
        
    taxis = np.linspace(0,simtime,simlen)
    
    #%%
            
    plt.figure()
    plt.plot(taxis,Tstack,'b--')
    plt.plot(taxis,Tano,'r--')
    plt.plot(taxis,Tcat,'g--')
    plt.legend(["Stack","Anolyte", "Catholyte"])
    plt.ylabel('Temperature [$C^o$]')
    plt.xlabel('Time [hr]')   
    
