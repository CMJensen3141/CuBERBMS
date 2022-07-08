# -*- coding: utf-8 -*-
"""
Created on Tue Jun 28 12:45:38 2022

@author: Christian Møller Jensen
This file implements a novel state-space electrochemical model of a
copper redox flow battery
"""

import numpy as np
import scipy as sp
import scipy.linalg as spla

class CuRFB_EC:
    def __init__(self,c_init_tanks,c_init_cell):
        self.c_tank = c_init_tanks # Initial reactant species concentrations in tanks
        self.c_cell = c_init_cell # Initial reactant species concentration in cells
        self.V_cell = 0.001 # Cell volume
        self.V_tank = 1 # Electrolyte tank volume
        self.N = 1 # Number of cells in the stack
        self.z = 1 # Number of electrons exchanged by the reaction
        self.F = 96485 # Faraday constant
        self.R = 8.314 # Ideal gas constant
        self.S = 1 # Membrane surface area
        self.d = 1 # Membrane thickness
        self.k = np.ones([4,4])  # Diffusion coefficients (Fick's Law)
        self.alpha = np.array([[-1, 0, -1, -2,],[0, -1,2, 3],[3, 2, -1, 0],[-2, -1, 0, -1]]) # Species mass balance factors
        self.Ts = 1 # Model discretisation time
        self.CurrentSigns = np.array([1,-1,-1,1]).reshape(4,1)
        self.A = self.alpha*self.k
        self.E0 = 0.7 # Formal potential [V]
        
    def Model_Timestep(self,qa,qc,I):
        Q = spla.block_diag(np.diag([qa,qa]),np.diag([qc,qc]))
        flowpart = Q@(self.c_cell-self.c_tank)
        currentpart = 1/(self.z*self.F)*self.CurrentSigns
        diffusionpart = self.S/self.d*self.A@self.c_cell
        self.c_cell += self.Ts*(flowpart+currentpart+diffusionpart)/self.V_cell 
        self.c_tank += self.Ts*self.N*Q@(self.c_cell-self.c_tank)
        
    def Model_GetSOC(self):
        SOCvar = self.c_tank[0]/(self.c_tank[0]+self.c_tank[1])
        return SOCvar
        
    def Model_GetCells(self):
        return self.c_cell
    
    def Model_GetTanks(self):
        return self.c_tank
    
    def Model_GetVoltage(self,T_cell):
        voltvar = self.E0 + self.R*T_cell/self.F*np.log((self.c_tank[0]/self.c_tank[1])*(self.c_tank[2]/self.c_tank[3])) 
        return voltvar
        
#%% Unit tests

c_init_tanks = np.array([1,0.5,1,0.5],dtype = np.float64).reshape(4,1)
c_init_cell = np.zeros([4,1])
fooCU = CuRFB_EC(c_init_tanks, c_init_cell)

print(fooCU.Model_GetCells())
print(fooCU.Model_GetTanks())
fooCU.Model_Timestep(0.1,0.1,100)
print(fooCU.Model_GetCells())
print(fooCU.Model_GetTanks())
print("SOC is",fooCU.Model_GetSOC())
print("Cell voltage is", fooCU.Model_GetVoltage(25))