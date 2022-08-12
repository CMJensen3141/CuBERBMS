# -*- coding: utf-8 -*-
"""
Created on Thu May 19 23:13:20 2022

@author: Christian
"""

import numpy as np
from numba import njit, jit
import numba
import math

class GraphHydraulicModel:
    def __init__(self,Ts_extern,Ts_intern):
        self.H = np.array([[1,0,0,0,-1],[-1,1,0,0,0],[0,-1,1,0,0],[0,0,-1,1,0],[0,0,0,-1,1]]) # Define the incidence matrix
        self.G = np.array([0,0,0,0,1]).reshape(5,1) # Define the pump mapping matrix
        self.H_bar = self.H[1:,...] # Define the reduced incidence matrix
        self.H_bar_C = self.H_bar[:,1].reshape(4,1) # Define the chord section of the reduced incidence matrix
        self.H_bar_T = self.H_bar[:,[0,2,3,4]] # Define the tree section of the reduced 
        self.B = np.array([1]) # Instantiate the loop matrix
        self.B = np.append(self.B,np.array([ -self.H_bar_C.T @ np.linalg.inv(self.H_bar_T).T])).reshape(1,5) # Calculate the loop matrix
        self.J = np.array([[0,0,0,0,0],[0,0.0023,0,0,0],[0,0,0.0047,0,0],[0,0,0,0.0047,0],[0,0,0,0,0]]) # Define inertia matrix
        self.Kp = np.array([0,0,0.0026,0.0026,0.0020]).reshape(5,1) # Define the pipe resistance matrix
        self.q = np.array([0,0,0,0,0]).reshape(5,1) # Define the flow vector
        self.qC = np.array([0]).reshape(1,1) # Define the chord flow vector
        self.dP = np.array([0,0,0,0,0]).reshape(5,1) # Define the pressure vector
        self.Ts_extern = Ts_extern # Define the overall sampling time
        self.Ts_intern = Ts_intern # Define the internal sampling time to avoid stiffness issues
        self.KozCar = 5 #Kozeny-Carman constant
        self.length = 0.26 # Electrode length (m)
        self.area = 0.0875 # Electrode area (m^2)
        self.viscosity = 4.928e-8 # Fluid viscosity (bar*s)
        self.fibdiam =  11e-6 # Mean fiber diameter (m)
        self.porosity = 0.68 # Electrode porosity (unitless)
        
        self.perm = (self.fibdiam**2*self.porosity**3)/(self.KozCar*(1-self.porosity)**2) # Calculate the permeability coefficient
        
        self.Rcell = 0.2*(self.viscosity*self.length)/(self.perm*self.area) # Calculate the hydraulic resistance from Darcy's Law [unit = bar/(m^3/s)]

        
        
        
        
        self.NumbaStep = self.NumbaGenerator()
        

        
    def PipeFun(self,q):
        res = np.multiply(self.Kp,q)
        res = np.multiply(res,abs(q))
        return np.array(res)
    
    def StackFun(self,q):
        if q[1] == 0:
            res = 0
        else:
            res = (39.089+(q*0.06)*27.868 + np.multiply(0.063*abs(q*0.06),(q*0.06)))/1000
        return np.array(res)

    def PumpFun(self,q,w):
        res = -(0.0001*(w*w)+0.0004*w*q-0.0355*abs(q)*q)
        return np.array(res)
    
    def TimeStep(self,q,w):
        if w == 0:
            self.q = np.zeros([5,1])
            self.dP = np.zeros([5,1])
        else:
            self.rp = self.PipeFun(q) # Calculate the pipe resistances
            self.rs = np.multiply(np.array([0,1,0,0,0]).reshape(5,1),self.StackFun(q)) # Calculate the stack resistance
            self.r_pump = self.G @ self.PumpFun(q[-1],w) # Calculate the pump resistance
            self.r_pump = self.r_pump.reshape(5,1) # Fit the pump resistance into the right size
            self.qC_dot = -np.linalg.inv((self.B @ self.J @ self.B.T).reshape(1,1)) * (self.B @ (self.rp+self.rs+self.r_pump)) # Differential equation describing flows
            self.qC += self.qC_dot*self.Ts # Euler timestep
            self.q = self.B.T @ self.qC # Calculate the flow vector
            self.dP = self.J@self.B.T*self.qC_dot*self.Ts + self.rp + self.rs + self.r_pump # Calculate the pressure vector
            
    def NumbaGenerator(self):
        Kp = self.Kp.astype(np.float64); B = self.B.astype(np.float64); J = self.J.astype(np.float64); G = self.G.astype(np.float64); Ts_int = self.Ts_intern; Ts_ext = self.Ts_extern; # Local copies of necessary static variables
        BT = B.T.astype(np.float64); S = np.array([0,1,0,0,0]).reshape(5,1); Rcell = self.Rcell
        
        if math.ceil(Ts_ext/Ts_int) > 1:
            numsteps = math.ceil(Ts_ext/Ts_int)
        else:
            numsteps = 1
        
        BJBT = -np.linalg.inv((B @ J @ B.T).reshape(1,1)) # Pre-calculate and hardcode inverted matrix to save computational effort
        
        
        @njit
        def NumbaTimestep(qC,q,w):
                if w == 0:
                    qC = np.zeros((1,1))  # Has to be a tuple because Numba is fucking weird
                    q = np.zeros((5,1))
                    dP = np.zeros((5,1))
                else:
                    qC = qC.astype(np.float64)
                    q = q.astype(np.float64)
                    for ii in range(0,numsteps):
                        rp = np.multiply(Kp,q)
                        # rs = (39.089+(q[1]*0.06)*27.868 + np.multiply(0.063*np.absolute(q[1]*0.06),(q[1]*0.06)))/1000 # Visblue polynomial from deliverable
                        rs = Rcell*q[1]/3600 # Using Darcy's Law and best guess at porous membrane resistance
                        rs = np.multiply(S,rs)
                        r_pump = G @ -(0.0001*(w*w)+0.0004*w*q[-1]-0.0355*np.absolute(q[-1])*q[-1])
                        r_pump = r_pump.reshape(5,1)
                        qC_dot = BJBT * (B @ (rp+rs+r_pump))
                        qC += qC_dot*Ts_int
                        q = BT @ qC
                        dP = J@BT*qC_dot*Ts_int + rp + rs + r_pump
                return qC , q , dP 
                    
        return NumbaTimestep            
        
    def GetVals(self):
        return self.qC, self.q, self.dP
    
    def SetVals(self,qC,q,dP):
        self.qC = qC 
        self.q = q 
        self.dP = dP

if __name__ == "__main__": # Unit tests, run only if executing this file as main window

    import matplotlib.pyplot as plt
    
    fooMod = GraphHydraulicModel(10,0.01)
    
    simtime = 10
    
    simlen = round(simtime*3600/fooMod.Ts_extern)
    taxis = np.linspace(0,simtime,simlen)
    
    qCvec = np.zeros([1,simlen])
    qvec = np.zeros([5,simlen])
    pvec = np.zeros([5,simlen])
    # vals = fooMod.GetVals()
    
    for ii in range(0,simlen):
        vals = fooMod.GetVals()
        qCvec[:,ii] = 16.7*vals[0].flatten()
        qvec[:,ii] = 16.7*vals[1].flatten()
        pvec[:,ii] = 1000*vals[2].flatten()
        if ii > simlen/2:
            out = fooMod.NumbaStep(vals[0],vals[1],float(100))
        elif ii > simlen/4:
            out = fooMod.NumbaStep(vals[0],vals[1],float(40))
        else:
            out = fooMod.NumbaStep(vals[0],vals[1],float(0))
        fooMod.SetVals(out[0], out[1], out[2])
        # vals = out
           
    #%% 
            
    plt.figure()
    for ii in range(0,5):
        plt.plot(taxis,qvec[ii,:],'--')
    plt.legend(['Pipe 1','Stack','Pipe 2','Pipe 3','Pump'])   
    plt.xlabel('Time [hr]')
    plt.ylabel('Flow [l/min]')
    
    plt.figure()
    for ii in range(0,5):
        plt.plot(taxis,pvec[ii,:],'--')
    plt.legend(['Pipe 1','Stack','Pipe 2','Pipe 3','Pump']) 
    plt.xlabel('Time [hr]')
    plt.ylabel('Differential pressure [millibar]')    