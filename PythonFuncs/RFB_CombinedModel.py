# -*- coding: utf-8 -*-
"""
Created on Mon Aug  8 09:41:20 2022

@author: chris
"""

import RFB_ElectroChemModel as ElectricModel
import RFB_GraphHydraulicModel as HydraulicModel
import RFB_FlowThermalModel as ThermalModel
import numpy as np

class RFB_CombinedModel:
    def __init__(self,Ts_ext,Rstack,T_ambient,c_total):
        c_init_tanks = np.array([0,2500,2500,0],dtype = np.float64).reshape(4,1)
        c_init_cell = np.array([0,0,0,0],dtype = np.float64).reshape(4,1)
        
        self.EModel = ElectricModel.CuRFB_EC(c_init_tanks, c_init_cell, Ts_ext, 1)
        
        self.AnoModel = HydraulicModel.GraphHydraulicModel(Ts_ext, 0.05)
        self.CatModel = HydraulicModel.GraphHydraulicModel(Ts_ext, 0.05)
        
        self.TModel = ThermalModel.RFB_FD_ThermalModel(Rstack, T_ambient, Ts_ext, 1)
        
        
    def Model_Timestep(self,w_ano,w_cat,I):
        
        cubichrtocubicsec = 1/3600
        
        #Simulate the anolyte circuit
        aVals  = self.AnoModel.GetVals()
        aOut = self.AnoModel.NumbaStep(aVals[0],aVals[1],float(w_ano))
        self.AnoModel.SetVals(aOut[0],aOut[1],aOut[2])
        qAno = float(aOut[0].flatten())*cubichrtocubicsec # Convert m3/hr to m3/s
        
        cVals  = self.CatModel.GetVals()
        cOut = self.CatModel.NumbaStep(cVals[0],cVals[1],float(w_cat))
        self.CatModel.SetVals(cOut[0],cOut[1],cOut[2])
        qCat = float(cOut[0].flatten())*cubichrtocubicsec # Convert m3/hr to m3/s
        
        
        tVals = np.array([self.TModel.GetTemps()],dtype=np.float64).flatten()
        tOut = self.TModel.NumbaStep(tVals[0],tVals[1],tVals[2],self.TModel.T_air,qAno,qCat,I)
        self.TModel.SetTemps(tOut[0], tOut[1], tOut[2])
        
        cellVals = self.EModel.GetCells()
        tankVals = self.EModel.GetTanks()
        eOut = self.EModel.NumbaStep(qAno, qCat, I, cellVals, tankVals)
        self.EModel.SetCells(eOut[0])
        self.EModel.SetTanks(eOut[1])
        
    def GetCellsConc(self):
        return self.EModel.GetCells()
    
        
    def GetTanksConc(self):
        return self.EModel.GetTanks()
        
        
    def GetTemps(self):
        return self.TModel.GetTemps()
    
    def GetVoltage(self):
        return self.EModel.GetVoltage(self.TModel.T_s)*self.EModel.N
        
    def GetSOC(self):
        return self.EModel.GetSOC()
      
    def GetAnoFlows(self):
        cubichrtolitersec = 1/3.6
        out = self.AnoModel.GetVals()
        return out[0]*cubichrtolitersec
        
    def GetCatFlows(self):
        cubichrtolitersec = 1/3.6 
        out = self.CatModel.GetVals()
        return out[0]*cubichrtolitersec
        
    def GetAnoPressures(self):
        out = self.AnoModel.GetVals() # Returns a tuple
        return out[2] # Returns an array of pressures
    
    def GetAnoPressure_Stack(self):
        out = self.GetAnoPressures()
        return out[1]*1000 # return stack pressure in millibar
    
    def GetAnoPressure_Pump(self):
        out = self.GetAnoPressures()
        return out[-1]*1000 # return stack pressure in millibar
        
    def GetCatPressures(self):
        out = self.CatModel.GetVals() # Returns a tuple
        return out[2] #Returns array of pressures
    
    def GetCatPressure_Stack(self):
        out = self.GetCatPressures()
        return out[1]*1000 # return stack pressure in millibar
    
    def GetCatPressure_Pump(self):
        out = self.GetCatPressures()
        return out[-1]*1000 # return stack pressure in millibar
        
        
if __name__ == "__main__": # Unit tests, run only if executing this file as main window
        
    import matplotlib.pyplot as plt
    
    Ts_sim = 100
    Rstack = 0.073
    c_total = 2500
    
    CombinedModel = RFB_CombinedModel(Ts_sim, Rstack, 30, c_total)
    
    simtime = 4
    simlen = round(simtime*3600/Ts_sim)
    taxis = np.linspace(0,simtime,simlen)
            
    
    ano_qCvec = np.zeros([1,simlen])
    ano_qvec = np.zeros([5,simlen])
    ano_pvec = np.zeros([5,simlen])
    cat_qCvec = np.zeros([1,simlen])
    cat_qvec = np.zeros([5,simlen])
    cat_pvec = np.zeros([5,simlen])
    Temps = np.empty([3,simlen])
    c_tank = np.empty([4,simlen])
    c_cell = np.empty([4,simlen])
    Vbat = np.empty([1,simlen])
    SOCbat = np.empty([1,simlen])
    
    for ii in range(0,simlen):
        if ii > simlen/2:
            w_ano = 100
            w_cat = 50
            I = -150
        else:
            w_ano = 50
            w_cat = 100
            I = 150
        CombinedModel.Model_Timestep(w_ano,w_cat,I)
        aVals = CombinedModel.AnoModel.GetVals()
        ano_qCvec[:,ii] = 16.7*aVals[0].flatten()
        ano_qvec[:,ii] = 16.7*aVals[1].flatten()
        ano_pvec[:,ii] = 1000*aVals[2].flatten()
        cVals = CombinedModel.CatModel.GetVals()
        cat_qCvec[:,ii] = 16.7*cVals[0].flatten()
        cat_qvec[:,ii] = 16.7*cVals[1].flatten()
        cat_pvec[:,ii] = 1000*cVals[2].flatten()
        
        tVals = CombinedModel.TModel.GetTemps()
        Temps[:,ii] = tVals
        
        c_tank[:,ii] = CombinedModel.EModel.GetTanks().flatten()
        c_cell[:,ii] = CombinedModel.EModel.GetCells().flatten()
        Vbat[:,ii] = CombinedModel.EModel.GetVoltage(Temps[0,ii])*CombinedModel.EModel.N
        SOCbat[:,ii] = CombinedModel.EModel.GetSOC()
    
        
    plt.figure()
    plt.subplot(2,1,1)
    for ii in range(0,5):
        plt.plot(taxis,ano_qvec[ii,:],'--')
    plt.legend(['Pipe 1','Stack','Pipe 2','Pipe 3','Pump'])   
    plt.xlabel('Time [hr]')
    plt.ylabel('Anolyte flows [l/min]')
    plt.ylim(0,100)
    plt.subplot(2,1,2)
    for ii in range(0,5):
        plt.plot(taxis,cat_qvec[ii,:],'--')
    plt.legend(['Pipe 1','Stack','Pipe 2','Pipe 3','Pump'])   
    plt.xlabel('Time [hr]')
    plt.ylabel('Catholyte flows [l/min]')
    plt.ylim(0,100)
    
    
    plt.figure()
    for ii in range(0,5):
        plt.plot(taxis,ano_pvec[ii,:],'--')
    plt.legend(['Pipe 1','Stack','Pipe 2','Pipe 3','Pump']) 
    plt.xlabel('Time [hr]')
    plt.ylabel('Anolyte circuit differential pressures [millibar]')      
    
    plt.figure()
    plt.plot(taxis,Temps[0,:].flatten(),'b--')
    plt.plot(taxis,Temps[1,:].flatten(),'r-.')
    plt.plot(taxis,Temps[2,:].flatten(),'g--')
    plt.legend(["Stack","Anolyte", "Catholyte"])
    plt.ylabel('Temperature [$C^o$]')
    plt.xlabel('Time [hr]')         
    
            
    plt.figure()
    for ii in range(0,c_tank.shape[0]):
        if ii < 2:
            plt.plot(taxis,c_tank[ii,:].flatten())
        else:
            plt.plot(taxis,c_tank[ii,:].flatten(),'--')
    plt.legend(['Anolyte 1','Anolyte 2','Catholyte 1','Catholyte 2'])
    plt.ylabel('Species concentrations, tanks')
    plt.xlabel('Time [hr]')  
    
    plt.figure()
    for ii in range(0,c_cell.shape[0]):
        if ii < 2:
            plt.plot(taxis,c_cell[ii,:].flatten())
        else:
            plt.plot(taxis,c_cell[ii,:].flatten(),'--')
    plt.legend(['Anolyte 1','Anolyte 2','Catholyte 1','Catholyte 2'])
    plt.ylabel('Species concentrations, cells')
    plt.xlabel('Time [hr]')  
    
    plt.figure()
    plt.plot(taxis,Vbat.flatten())
    plt.ylabel('Battery voltage')
    plt.xlabel('Time [hr]')  
    
    plt.figure()
    plt.plot(taxis,SOCbat.flatten())
    plt.ylabel('Battery SOC')
    plt.xlabel('Time [hr]') 