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


literstocubic = 1.66666667*10**-5

Ts = 0.1

Tmodel = RFBFTM.RFB_FD_ThermalModel(0.25,Ts)
Hmodel = GRFMOD.GraphHydraulicModel(Ts)
simtime = 1

#%%

simlen = round(simtime*3600/Hmodel.Ts)
taxis = np.linspace(0,simtime,simlen)

qvec = np.empty([5,simlen])
pvec = np.empty([5,simlen])
vals = np.empty([5])
current = np.empty(simlen)
SOC = np.empty(simlen)
U = np.empty(simlen)
T = np.empty(simlen)
Ttank = np.empty(simlen)
w = np.empty(simlen)
soctemp = 0
Utemp = 0
myBMS.Mode = 1

for ii in range(0,simlen):
    vals = Hmodel.GetVals()
    qvec[:,ii] = 16.7*vals[0].flatten()
    pvec[:,ii] = 1000*vals[1].flatten()
    T[ii] = Tmodel.GetTemps()[0]
    Ttank[ii] = Tmodel.GetTemps()[1]
    if ii < simlen/2:
        current[ii] = 100
        soctemp += 90/(simlen/2)
        Utemp += 50/(simlen/2)
        SOC[ii] = soctemp
        U[ii] = Utemp
        CB.setMeasurements(myBMS,current[ii],U[ii],U[ii]*current[ii],pvec[1,ii],SOC[ii],T[ii],qvec[1,ii])
        w[ii] = CB.calcPumpRef(myBMS)
        w[ii] = min(max(w[ii],0),100)
        # w[ii] = 50
        Hmodel.TimeStep(Hmodel.q,w[ii])
        Tmodel.ModelTimestep(literstocubic*abs(Hmodel.q[2]), literstocubic*abs(Hmodel.q[2]), current[ii])
    else:
        myBMS.mode = -1
        current[ii] = -100
        soctemp -= 90/(simlen/2)
        U[ii] = -U[2*round(simlen/2)-ii]
        SOC[ii] = soctemp
        CB.setMeasurements(myBMS,current[ii],U[ii],U[ii]*current[ii],pvec[2,ii],SOC[ii],T[ii],qvec[2,ii])
        w[ii] = CB.calcPumpRef(myBMS)
        w[ii] = min(max(w[ii],0),100)
        # w[ii] = 100
        Hmodel.TimeStep(Hmodel.q,w[ii])
        Tmodel.ModelTimestep(literstocubic*abs(Hmodel.q[2]), literstocubic*abs(Hmodel.q[2]), current[ii])

#%% 
       
plt.figure()
for ii in range(0,5):
    plt.plot(taxis,qvec[ii,:],'--')
# plt.plot(tax)
plt.legend(['Pipe 1','Stack','Pipe 2','Pipe 3','Pump'])
plt.ylabel('Flow') 
plt.xlabel('Time [hr]') 

plt.figure()
for ii in range(0,5):
    plt.plot(taxis,pvec[ii,:],'--')
plt.legend(['Pipe 1','Stack','Pipe 2','Pipe 3','Pump'])
plt.ylabel("Differential pressure [mbar]")
plt.xlabel("Time [hr]")    

plt.figure()
plt.plot(taxis,SOC)

plt.figure()
plt.plot(taxis,current)

plt.figure()
plt.plot(taxis,U)

plt.figure()
plt.plot(taxis,T)
plt.plot(taxis,Ttank,'r')
plt.xlabel('Time [hr]')
plt.ylabel('Temperature [$^\circ$C]')
plt.legend(['Stack','Tank'])  

plt.figure()
plt.plot(taxis,w)
plt.xlabel('Time [hr]')
plt.ylabel('Pump speed [% PWM]')
