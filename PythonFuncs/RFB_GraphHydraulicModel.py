# -*- coding: utf-8 -*-
"""
Created on Thu May 19 23:13:20 2022

@author: Christian
"""

import numpy as np

class GraphHydraulicModel:
    def __init__(self,Ts):
        self.H = np.array([[1,0,0,0,-1],[-1,1,0,0,0],[0,-1,1,0,0],[0,0,-1,1,0],[0,0,0,-1,1]])
        self.G = np.array([0,0,0,0,1]).reshape(5,1)
        self.H_bar = self.H[1:,...]
        self.H_bar_C = self.H_bar[:,1].reshape(4,1)
        self.H_bar_T = self.H_bar[:,[0,2,3,4]]
        self.B = np.array([1])
        self.B = np.append(self.B,np.array([ -self.H_bar_C.T @ np.linalg.inv(self.H_bar_T).T])).reshape(1,5)
        self.J = np.array([[0,0,0,0,0],[0,0.0023,0,0,0],[0,0,0.0047,0,0],[0,0,0,0.0047,0],[0,0,0,0,0]])
        self.Kp = np.array([0,0,0.0026,0.0026,0.0020]).reshape(5,1)
        self.q = np.array([0,0,0,0,0]).reshape(5,1)
        self.qC = np.array([0]).reshape(1,1)
        self.dP = np.array([0,0,0,0,0]).reshape(5,1)
        self.Ts = Ts
        
    def PipeFun(self,q):
        res = np.multiply(self.Kp,q)
        res = np.multiply(res,abs(q))
        return np.array(res)
    
    def StackFun(self,q):
        res = (39.089+(q*0.06)*27.868 + np.multiply(0.063*abs(q*0.06),(q*0.06)))/1000
        return np.array(res)

    def PumpFun(self,q,w):
        res = -(0.0001*w**2+0.0004*w*q-0.0355*abs(q)*q)
        return np.array(res)
    
    def TimeStep(self,q,w):
        self.rp = self.PipeFun(q)
        self.rs = np.multiply(np.array([0,1,0,0,0]).reshape(5,1),self.StackFun(q))
        self.r_pump = self.G @ self.PumpFun(q[-1],w)
        self.r_pump = self.r_pump.reshape(5,1)
        self.qC_dot = -np.linalg.inv((self.B @ self.J @ self.B.T).reshape(1,1)) * (self.B @ (self.rp+self.rs+self.r_pump))
        self.qC = self.qC + self.qC_dot*self.Ts
        self.q = self.B.T @ self.qC
        self.dP = self.J@self.B.T*self.qC_dot*self.Ts + self.rp + self.rs + self.r_pump
        
    def GetVals(self):
        return self.q, self.dP

#%% Test that class works correctly

import RFB_GraphHydraulicModel as GRFMOD
import matplotlib.pyplot as plt

fooMod = GRFMOD.GraphHydraulicModel(0.01)

simlen = round(60/fooMod.Ts)

qvec = np.empty([5,simlen])
pvec = np.empty([5,simlen])
vals = np.empty([5])


for ii in range(0,simlen):
    vals = fooMod.GetVals()
    qvec[:,ii] = 16.7*vals[0].flatten()
    pvec[:,ii] = 1000*vals[1].flatten()
    if ii < simlen/2:
        fooMod.TimeStep(fooMod.q,100)
    else:
        fooMod.TimeStep(fooMod.q,50)
       
plt.figure()
for ii in range(0,5):
    plt.plot(qvec[ii,:],'--')
plt.legend(['Pipe 1','Stack','Pipe 2','Pipe 3','Pump'])   

plt.figure()
for ii in range(0,5):
    plt.plot(pvec[ii,:],'--')
plt.legend(['Pipe 1','Stack','Pipe 2','Pipe 3','Pump'])     