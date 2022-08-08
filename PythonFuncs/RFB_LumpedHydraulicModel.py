# -*- coding: utf-8 -*-
"""
Created on Wed Aug 25 15:04:49 2021


This module implements a lumped-parameter hydraulic model of a redox flow battery stack.
The model is particularly concerned with estimating differential pressure across the stack electrode for control purposes. 


@author: Christian Møller Jensen, Bac. Eng. EE., Aarhus University.

Acknowledgements: The model in this module is based on work by Ra and Bhattacharjee, see doi.org/10.1002/ente.202000708. Half-cell physical
parameters in test case are based on Zhang et al., see Table 1 in doi.org/10.1016/j.jpowsour.2015.04.169.

Parameter values from Zhang et al. are listed as default values in the init function.     
"""

class RFB_LumpedHydraulicModel:
    def __init__(self,viscosity,permeability,density,length,width,depth,Nseries):
        self.permeability = permeability
        self.viscosity = viscosity
        self.density = density
        self.length = length
        self.width = width
        self.depth = depth
        self.Nseries = Nseries
        self.StackPressure = 0 
        RFB_LumpedHydraulicModel.__DarcyResistance__(self)


    def __DarcyResistance__(self):
        self.Rcell = (self.viscosity*self.length)/(self.permeability*self.width*self.depth)
    
    def Model_Timestep(self,FlowRate):
        FlowRate = FlowRate*1e-3
        self.StackPressure = 2*(FlowRate*self.Rcell)/(0.7*self.Nseries)
    
    def Get_StackPressure(self,*Unit):
        if Unit == "Pascal":
            return self.StackPressure
        else:
            return self.StackPressure*1e-5
        
    def Get_DarcyResistance(self):
        return self.Rcell    
    
    
if __name__ == "__main__": # Unit tests, run only if executing this file as main window

    import ControlFuncs as CF
    import matplotlib.pyplot as plt
    import numpy as np
    
    viscosity = 0.006
    permeability = 1.685-10
    density = 1400
    length = 0.26
    width = 0.30
    depth = 3e-03
    Nseries = 15
    
    PascalToBar = 1e-5
    
    FlowController = CF.PID(0.005,0.001,0,0,0.1,0)
    
    
    LumpedModel = RFB_LumpedHydraulicModel(viscosity,permeability,density,length,width,depth,Nseries)
    
    print("Stack Darcy resistance is " + str(LumpedModel.Get_DarcyResistance()))
    
    FlowRate = 0
    
    dP = LumpedModel.Get_StackPressure()
    numiters = 1000
    
    print("Differential pressure is " + str(dP) + " pascal")
    print("Differential pressure is " + str(dP*PascalToBar) + " bar" )
    
    dP = np.empty(numiters)
    Q = np.empty(numiters)
    
    for ii in range(0,numiters):
        FlowRate = FlowController.run(ii,0.5,LumpedModel.Get_StackPressure(),FlowRate)
        LumpedModel.Model_Timestep(FlowRate)
        dP[ii] = LumpedModel.Get_StackPressure()
        Q[ii] = FlowRate
    
    plt.figure()
    plt.plot(dP,'b--')
    plt.legend(["Stack differential pressure [bar]"])
    
    plt.figure()
    plt.plot(Q,'r')
    "Electrolyte flow [L/s]"