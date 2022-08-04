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
from numba import njit, jit
import math
import numba

class CuRFB_EC:
    def __init__(self,c_init_tanks,c_init_cell,Ts_extern,Ts_intern):
        self.c_tank = c_init_tanks.reshape(4,1) # Initial reactant species concentrations in tanks
        self.c_cell = c_init_cell.reshape(4,1) # Initial reactant species concentration in cells
        self.V_cell = 9.5*10**-4 # Cell volume
        self.V_tank = 0.4 # Electrolyte tank volume
        self.N = 19 # Number of cells in the stack
        self.z = 1 # Number of electrons exchanged by the reaction
        self.F = 96485 # Faraday constant
        self.R = 8.314 # Ideal gas constant
        self.S = 0.15 # Membrane surface area5
        self.d = 1.27*10**-4 # Membrane thickness
        self.k = np.array([8.768, 3.221, 6.825, 5.896])*10**-12 # Diffusion coefficients (Fick's Law)
        self.alpha = np.array([[-self.k[0], 0, -self.k[2], -2*self.k[3],],[0, -self.k[1],2*self.k[2], 3*self.k[3]],[3*self.k[0], 2*self.k[1], -self.k[2], 0],[-2*self.k[0], -self.k[1], 0, -self.k[3]]],dtype = np.float64) # Species mass balance factors
        self.Ts_extern = Ts_extern # Discretisation time of outer simulation loop
        self.Ts_intern = Ts_intern # Discretisation time of model proper
        self.CurrentSigns = np.array([1,-1,-1,1]).reshape(4,1)
        self.A = self.alpha
        # self.E0 = 0.7 # Formal potential [V] of copper
        self.E0 = 1.259 # Formal potential [V] of vanadium
        self.Numba_Timestep = self.NumbaGenerator()
        self.NumbaVoltage = self.GetVoltageGenerator()
        
    def Model_Timestep(self,qa,qc,I):
        Q = spla.block_diag(np.diag([qa,qa]),np.diag([qc,qc]))
        flowpart = Q@(self.c_tank-self.c_cell)
        currentpart = 1/(self.z*self.F)*self.CurrentSigns*I
        diffusionpart = self.S/self.d*self.A@self.c_cell
        self.c_cell += self.Ts_intern*(flowpart+currentpart+diffusionpart)/self.V_cell 
        self.c_tank += self.Ts_intern*(self.N*Q@(self.c_cell-self.c_tank))/self.V_tank
        
    def NumbaGenerator(self):
        z = self.z; F = self.F; CurrentSigns = self.CurrentSigns; S = self.S; d = self.d; A = self.A; N = self.N; V_tank = self.V_tank; V_cell = self.V_cell; Ts_intern = self.Ts_intern; Ts_extern = self.Ts_extern
        
        numsteps = math.ceil(Ts_extern/Ts_intern)
        
        @njit
        def NumbaTimestep(qa,qc,I,c_cell,c_tank):
            Q = np.array([[qa, 0, 0, 0,],[0, qa, 0, 0],[qc, 0, 0, 0],[0, 0, 0, qc]],dtype = np.float64)
            currentpart = 1/(z*F)*CurrentSigns*I
            for ii in range(0,numsteps):
                flowpart = Q@(c_tank-c_cell).reshape(4,1)
                diffusionpart = S/d*A@c_cell.reshape(4,1)
                c_cell += Ts_intern*(flowpart+currentpart+diffusionpart)/(2*V_cell) 
                c_tank += Ts_intern*(N*Q@(c_cell-c_tank))/V_tank
            for jj in range(0,4):
                if c_cell[jj] <= 0:
                    c_cell[jj] = 0
                if c_tank[jj] <= 0:
                    c_tank[jj] = 0     

            return c_cell, c_tank
        
        return NumbaTimestep
        
    def Model_GetSOC(self):
        SOCvar = self.c_tank[0]/(self.c_tank[0]+self.c_tank[1])
        return SOCvar
        
    def Model_GetCells(self):
        return self.c_cell
    
    def Model_SetCells(self,c_cell):
        self.c_cell = c_cell.reshape(4,1)
    
    def Model_GetTanks(self):
        return self.c_tank
    
    def Model_SetTanks(self,c_tank):
        self.c_tank = c_tank.reshape(4,1)
        
    def GetVoltageGenerator(self):
        E0 = self.E0; R = self.R; F = self.F;
        
        @njit
        def NumbaVoltage(T_cell,c_tank):
            c_tank_stab = c_tank+0.1
            voltvar = E0 + R*T_cell/F*np.log((c_tank_stab[0]/c_tank_stab[1])*(c_tank_stab[2])/c_tank_stab[3])
            return voltvar
        return NumbaVoltage
    
    def Model_GetVoltage(self,T_cell):
        return self.NumbaVoltage(T_cell,self.c_tank)
        
            
#%% Unit tests

Ts_ext = 100
Ts = 1
c_init_tanks = np.array([200,2300,2300,200],dtype = np.float64).reshape(4,1)
c_init_cell = np.array([0,0,0,0],dtype = np.float64).reshape(4,1)
# c_init_cell = c_init_tanks
fooCU = CuRFB_EC(c_init_tanks, c_init_cell, Ts_ext, Ts)


#%%

import numpy as np
import matplotlib.pyplot as plt

literstocubic = 1.66666667*10**-5


simtime = 24 # Simulation time in hours

simlen = round((simtime*3600)/Ts_ext) 

taxis = np.linspace(0,simtime,simlen).reshape(simlen,1)

c_tank = np.empty([4,simlen])
c_cell = np.empty([4,simlen])
Vbat = np.empty([1,simlen])
SOCbat = np.empty([1,simlen])

flow1 = 10*literstocubic
flow2 = 10*literstocubic
I_d = -150
I_c = 150

for ii in range(0,simlen):
    cellvals = fooCU.Model_GetCells()
    tankvals = fooCU.Model_GetTanks()
        
    if ii < simlen/2:
        if SOCbat[:,ii-1] < 0.8:
            out = fooCU.Numba_Timestep(flow1, flow1, I_c, cellvals, tankvals)
        else:
            out = fooCU.Numba_Timestep(flow1, flow1, 0, cellvals, tankvals)
    else:
        if SOCbat[:,ii-1] > 0.2:
            out = fooCU.Numba_Timestep(flow2, flow2, I_d, cellvals, tankvals)
        else:
            out = fooCU.Numba_Timestep(flow2, flow2, 0, cellvals, tankvals)
    fooCU.Model_SetCells(out[0])
    fooCU.Model_SetTanks(out[1])
    c_tank[:,ii] = fooCU.Model_GetTanks().flatten()
    c_cell[:,ii] = fooCU.Model_GetCells().flatten()
    Vbat[:,ii] = fooCU.Model_GetVoltage(25+273)
    SOCbat[:,ii] = fooCU.Model_GetSOC()

    
    


#%%

dims = taxis.shape
        
plt.figure()
for ii in range(0,c_tank.shape[0]):
    plt.plot(taxis,c_tank[ii,:].reshape(dims[0],dims[1]))
plt.legend(['Anolyte 1','Anolyte 2','Catholyte 2','Catholyte 1'])
plt.ylabel('Species concentrations, tanks')
plt.xlabel('Time [hr]')  

plt.figure()
for ii in range(0,c_cell.shape[0]):
    plt.plot(taxis,c_cell[ii,:].reshape(dims[0],dims[1]))
plt.legend(['Anolyte 1','Anolyte 2','Catholyte 1','Catholyte 2'])
plt.ylabel('Species concentrations, cells')
plt.xlabel('Time [hr]')  

plt.figure()
plt.plot(taxis,Vbat.reshape(dims[0],dims[1]))
plt.ylabel('Battery voltage')
plt.xlabel('Time [hr]')  

plt.figure()
plt.plot(taxis,SOCbat.reshape(dims[0],dims[1]))
plt.ylabel('Battery SOC')
plt.xlabel('Time [hr]') 