# -*- coding: utf-8 -*-
"""
Created on Fri Jun 24 14:08:31 2022

@author: chris
"""

import numpy as np
import RFB_CombinedModel as ComMod
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
BMS_PumpMax = 10   # Flow in [l/s]
BMS_PumpMin = 0.0  # Flow in [l/s]

Ts = 1
Rstack = 1.5
T_ambient = 25
c_total = 2500

BatteryModel = ComMod.RFB_CombinedModel(Ts,Rstack,T_ambient,c_total)

myBMS = CB.CuBMS(BMS_Lim_max, BMS_Lim_min, BMS_I_max, BMS_I_min, BMS_I_tog, BMS_PumpMax, BMS_PumpMin,BMS_EstMode,Ts)


literstocubic = 1.66666667*10**-5

simtime = 8

simlen = round(simtime*3600/Ts)
taxis = np.linspace(0,simtime,simlen)

qvec = np.zeros([1,simlen])
pvec = np.zeros([1,simlen])
SOC = np.zeros([1,simlen])
Ubat = np.zeros([1,simlen])
T_stack = np.zeros([1,simlen])
T_ano = np.zeros([1,simlen])
T_cat = np.zeros([1,simlen])
w_ano = np.zeros([1,simlen+1])
w_cat = np.zeros([1,simlen+1])
Icharge = np.zeros([1,simlen+1])
Pbat = np.zeros([1,simlen])
Pref = np.zeros([1,simlen])
Tref = np.zeros([1,simlen])
c_tank = np.empty([4,simlen])
c_cell = np.empty([4,simlen])
RestTicker = 0


myBMS.Mode = 1 # Set to charge initially
myBMS.setRefPower(2500) # 1kW initial power reference
myBMS.setRefTemp(55) # 55 degree desired stack temperature

for ii in range(0,simlen):
    Ubat[0,ii] = BatteryModel.GetVoltage()
    Pbat[0,ii] = Ubat[0,ii]*Icharge[0,ii]
    Pref[0,ii] = myBMS.getRefPower()
    Tref[0,ii] = myBMS.getRefTemp()
    qvec[0,ii] = BatteryModel.GetAnoFlows()
    pvec[0,ii] = BatteryModel.GetAnoPressure_Stack()
    SOC[0,ii] = BatteryModel.GetSOC()
    c_tank[:,ii] = BatteryModel.GetTanksConc().flatten()
    c_cell[:,ii] = BatteryModel.GetCellsConc().flatten()
    
    if SOC[0,ii] >= 0.8 and myBMS.Mode == 1:
        RestTicker += 1
        if RestTicker > 600/Ts:
            RestTicker = 0
            myBMS.Mode = -1 # Set to discharge
            myBMS.setRefPower(-2500)
    if SOC[0,ii] <= 0.1 and myBMS.Mode == -1:
        RestTicker += 1
        if RestTicker > 600/Ts:
            RestTicker = 0
            myBMS.Mode = 1 # Set to charge
            myBMS.setRefPower(2500)
    
    if ii > simlen/2:
        myBMS.setRefTemp(53)
    
    Temps = BatteryModel.GetTemps()
    T_stack[0,ii] = Temps[0]; T_ano[0,ii] = Temps[1]; T_cat[0,ii] = Temps[2];
    myBMS.setMeasurements(Icharge[0,ii], Ubat[0,ii], Pbat[0,ii], pvec[0,ii], SOC[0,ii], T_stack[0,ii], qvec[0,ii])
    Icharge[0,ii+1] = myBMS.DummyCurrentRef()
    w = myBMS.calcPumpRef(); 
    w_cat[0,ii+1] = w; w_ano[0,ii+1] = w;
    BatteryModel.Model_Timestep(w_ano[0,ii+1], w_cat[0,ii+1], Icharge[0,ii+1])
    
    
    
    

#%% 
       

plt.figure()
plt.subplot(3,1,1)
plt.plot(taxis,SOC.flatten())
plt.ylabel("SOC",rotation=0, ha='right')
plt.xlabel("Time [hr]")
plt.subplot(3,1,2)
for ii in range(0,2):
    plt.plot(taxis,c_cell[ii,:].flatten())
for ii in range(2,4):
    plt.plot(taxis,c_cell[ii,:].flatten(),'--')
plt.legend(["C1 cat","C1 ano","C2 cat","C2 ano"])
plt.ylabel("Cells [$mol/m^3$]",rotation=0, ha='right')
plt.xlabel("Time [hr]")
plt.subplot(3,1,3)
for ii in range(0,2):
    plt.plot(taxis,c_tank[ii,:].flatten())
for ii in range(2,4):
    plt.plot(taxis,c_tank[ii,:].flatten(),'--')
plt.legend(["C1 cat","C1 ano","C2 cat","C2 ano"])
plt.ylabel("Tanks [$mol/m^3$]",rotation=0, ha='right')
plt.xlabel("Time [hr]")

plt.figure()
plt.subplot(3,1,1)
plt.plot(taxis,Icharge[0,0:-1].flatten())
plt.ylabel("Current [A]")
plt.xlabel("Time [hr]")
plt.subplot(3,1,2)
plt.plot(taxis,Ubat.flatten())
plt.ylabel("Voltage [V]")
plt.xlabel("Time [hr]")
plt.subplot(3,1,3)
plt.plot(taxis,Pbat.flatten())
plt.plot(taxis,Pref.flatten(),'--r')
plt.legend(["Battery Power","Reference Power"])
plt.ylabel("Power [P]")
plt.xlabel("Time [hr]")

plt.figure()
plt.plot(taxis,T_stack.flatten())
plt.plot(taxis,T_ano.flatten(),'g')
plt.plot(taxis,Tref.flatten(),'--r')
plt.xlabel('Time [hr]')
plt.ylabel('Temperature [$^\circ$C]')
plt.legend(['Stack','Tank','Reference'])  

plt.figure()
plt.subplot(3,1,1)
plt.plot(taxis,w_ano[0,0:-1].flatten(),'--')
plt.xlabel('Time [hr]')
plt.ylabel('Pump [PWM]')
plt.subplot(3,1,2)
plt.plot(taxis,qvec.flatten(),'--')
plt.ylabel('Flow [L/s]') 
plt.xlabel('Time [hr]') 
plt.subplot(3,1,3)
plt.plot(taxis,pvec.flatten(),'--')
plt.ylabel("Stack dP [mbar]")
plt.xlabel("Time [hr]")  

plt.show()