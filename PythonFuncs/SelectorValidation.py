# -*- coding: utf-8 -*-
"""
Created on Fri Jun 24 14:08:31 2022

@author: chris
"""

import numpy as np
import RFB_GraphHydraulicModel as GRFMOD
import RFB_FlowThermalModel as RFBFTM
import matplotlib.pyplot as plt
Ts = 0.01

fooMod = RFBFTM.RFB_FD_ThermalModel(0.073,Ts)
fooMod = GRFMOD.GraphHydraulicModel(Ts)

simlen = round(10/fooMod.Ts)

qvec = np.empty([5,simlen])
pvec = np.empty([5,simlen])
vals = np.empty([5])

for ii in range(0,simlen):
    vals = fooMod.GetVals()
    qvec[:,ii] = 16.7*vals[0].flatten()
    pvec[:,ii] = 1000*vals[1].flatten()
    if ii > simlen/2:
        fooMod.TimeStep(fooMod.q,10)
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