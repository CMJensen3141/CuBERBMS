# -*- coding: utf-8 -*-
"""
Created on Sat Jul 31 17:29:28 2021

This module implements a hydraulic model of a redox flow battery stack. It will eventually support multiple flow patters and distribution
channel configurations.


@author: Christian Møller Jensen, Bac. Eng. EE., Aarhus University.

Acknowledgements: The model in this module is based on work by Wei et al., see 10.1016/j.jpowsour.2014.02.108.
Parameter values from Wei et al are used as default values in the init function.     
"""

class RFB_HydraulicModel:
    def __init__(self,FlowMode,CellModel,ManifoldModel,ChannelModel,FlowRate):
        self.FlowMode = FlowMode # 0 = no distribution channels
        self.CellModel = CellModel # Model of cell differential pressure
        self.ManifoldModel = ManifoldModel # Model of manifold differential pressure
        self.Channel = ChannelModel # Model of differential pressure across distribution channels
        self.CellPressure = 0
        self.ChannelPressure = 0
        self.StackPressure = 0
        
    def Calculate_StackPressure(self,FlowSum):
        self.StackPressure = (self.ManifoldModel.Rmi+self.ManifoldModel.Rmo)*FlowSum+self.CellPressure
                
    
    class RFB_CellModel():
        def __init__(self,Kp,Le,Ae,mu):
            self.Kp = Kp # Permeability coefficient calculated via the Kozeny-Carman equation.
            self.Le = Le # Electrode length
            self.Ae = Ae # Electrode area
            self.mu = mu # Dynamic viscosity of the fluid
            self.dPe = 0 # Differential pressure across the electrode 
        
        def Calculate_dPe(self,Q):
            self.dPe = (self.mu*self.Le*Q)/(self.Kp*self.Ae)
        
    
    class RFB_ManifoldModel():
        def __init__(self,Ksep,Kcom,fm,Lm,Dhm,rho,Am):
            self.Ksep = Ksep
            self.Kcom = Kcom
            self.fm = fm
            self.Lm = Lm
            self.Dhm = Dhm
            self.rho = rho
            self.Am = Am
            self.Calculate_ManifoldResistances(self)
            
        def Calculate_ManifoldResistances(self):
            self.Rmi = (self.Ksep + (self.fm*self.Lm)/self.Dhm)*self.rho/(2*self.A**2) 
            self.Rmo = (self.Kcom + (self.fm*self.Lm)/self.Dhm)*self.rho/(2*self.A**2) 
            
    
    class RFB_ChannelModel():
        def __init__(self,):
            
            