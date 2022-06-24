# -*- coding: utf-8 -*-
"""
Created on Fri Jun 24 14:08:31 2022

@author: chris
"""

import numpy as np
import RFB_GraphHydraulicModel as GRFMOD
import RFB_FlowThermalModel as RFBFTM
import matplotlib.pyplot as plt
import CuBMS as CB


# BMS parametre
BMS_mu     = 0.1 #Step size i RLS estimator
BMS_Lambda = 0.9999 #Forgetting factor in RLS
BMS_EstMode = 1   # Estimator mode for RLS estimator - 0 for differential measurements, 1 for direct


# Kp,Ki,Kd,beta,gamma,MV_init,Lim_max,Lim_min,I_max,I_min,I_tog,PumpMax,PumpMin
BMS_Lim_max = 2500 # Maximum limit (power)
BMS_Lim_min = -2500# Minimum limit (power)
BMS_I_max   = 150  # Maximum limit (current)
BMS_I_min   = -150 # Minimum limit (current)
BMS_I_tog   = 50   # Pump current threshold / toggle
BMS_PumpMax = 2    # Flow in [m^3/h]
BMS_PumpMin = 0.0  # Flow in [m^3/h]

myBMS = CB.CuBMS(BMS_Lim_max, BMS_Lim_min, BMS_I_max, BMS_I_min, BMS_I_tog, BMS_PumpMax, BMS_PumpMin,BMS_EstMode)


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