# -*- coding: utf-8 -*-
"""
Created on Thu May 19 23:13:20 2022

@author: Christian
"""

import numpy as np

class GraphHydraulicModel:
    def __init__(self,Ts):
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
        self.Ts = Ts # Define the sampling time
        
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
            self.qC = self.qC + self.qC_dot*self.Ts # Euler timestep
            self.q = self.B.T @ self.qC # Calculate the flow vector
            self.dP = self.J@self.B.T*self.qC_dot*self.Ts + self.rp + self.rs + self.r_pump # Calculate the pressure vector
        
    def GetVals(self):
        return self.q, self.dP

#%% Test that class works correctly

import RFB_GraphHydraulicModel as GRFMOD
import matplotlib.pyplot as plt

fooMod = GRFMOD.GraphHydraulicModel(0.01)

simlen = round(10/fooMod.Ts)

qvec = np.empty([5,simlen])
pvec = np.empty([5,simlen])
vals = np.empty([5])


for ii in range(0,simlen):
    vals = fooMod.GetVals()
    qvec[:,ii] = 16.7*vals[0].flatten()
    pvec[:,ii] = 1000*vals[1].flatten()
    if ii > simlen/2:
        fooMod.TimeStep(fooMod.q,0)
    elif ii > simlen/4:
        fooMod.TimeStep(fooMod.q,50)
    else:
        fooMod.TimeStep(fooMod.q,100)
       
plt.figure()
for ii in range(0,5):
    plt.plot(qvec[ii,:],'--')
plt.legend(['Pipe 1','Stack','Pipe 2','Pipe 3','Pump'])   

plt.figure()
for ii in range(0,5):
    plt.plot(pvec[ii,:],'--')
plt.legend(['Pipe 1','Stack','Pipe 2','Pipe 3','Pump'])     