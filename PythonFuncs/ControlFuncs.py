# -*- coding: utf-8 -*-
"""
Created on Mon Feb  8 02:32:31 2021

@author: Christian
"""

import numpy as np
import math as math

class PID_Backcal:
    def __init__(self,Kp,Ki,Kd,beta,gamma,MV_init):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.beta = beta
        self.gamma = gamma
        self.P = 0
        self.I = 0
        self.D = 0
        self.MV_init = MV_init
        self.MV = MV_init
        self.eDPrev = 0
        self.tPrev = 0
        
    def run(self,t,SP,PV,MV_actual):
        
        eD = self.gamma*SP-PV # Calculate weighted error
        dt = t-self.tPrev # Calculate time difference
        
        if self.Ki != 0:
            self.I = MV_actual-self.MV_init-self.P-self.D # Back-calculate integral part if there's an integral gain
        else:
            self.I = 0
        
        self.P = self.Kp*(self.beta*SP-PV) # Proportional part
        self.I += self.Ki*(SP-PV)*dt # Integral part (incremental)
        if dt == 0:
            self.D = 0
        else:
            self.D = self.Kd*(eD-self.eDPrev)/dt # Derivative part
        self.MV = self.P+self.I+self.D # Calculate output
        
        self.eDPrev = eD #Update error
        self.tPrev = t # Update time
        
        return self.MV
    
class PID:
    def __init__(self,Kp,Ki,Kd,poslim,neglim,MV_init):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.poslim = poslim
        self.neglim = neglim
        self.P = 0
        self.I = 0
        self.D = 0
        self.ePrev = 0
        self.tPrev = 0
        self.MV = MV_init
        
    def run(self,t,SP,PV):
    
        e = SP-PV
        dt = t-self.tPrev # Calculate time difference
        
        self.P = self.Kp*e # Proportional part
        if self.I > self.poslim:
            self.I = self.poslim
        elif self.I < self.neglim:
            self.I = self.neglim
        else:
            self.I += self.Ki*e*dt # Integral part (incremental)
        if dt == 0:
            self.D = 0
        else:
            self.D = self.Kd*(e-self.ePrev)/dt # Derivative part
        self.MV = self.P+self.I+self.D # Calculate output
        
        self.ePrev = e #Update error
        self.tPrev = t # Update time

        
        return self.MV
    
class CM_RLS:
    def __init__  (self,mu,Lambda,taps,Ts,EstMode):
        self.mu = mu
        self.Lambda = Lambda
        self.Theta = np.zeros([taps,1])
        self.Hessian = 1E3*np.eye(taps,taps)
        self.alpha = 0
        self.k = np.zeros([taps,1])
        self.Phi = np.zeros([taps,1])
        self.y_est = 0
        self.kappa = 0
        self.Ts = Ts
        self.Rs = 0.1
        self.Rp = 0.1
        self.Cp = 1000
        self.OCVhat = 0
        self.Beta = 0
        self.V_last = np.array(1) 
        self.V_last_last = np.array(1)
        self.I_last = np.array(1)
        self.I_last_last = np.array(1)
        self.EstMode = EstMode
        self.count = 0
        
    def RLS_update(self,phi_U,phi_I,d):
        self.Phi = np.concatenate([phi_U,phi_I],0)
        self.y_est = self.Phi.T@self.Theta
        self.alpha = d-self.y_est
        self.k = (self.Hessian@self.Phi)/(self.Lambda+self.Phi.T@self.Hessian@self.Phi)
        self.P = 1/self.Lambda*(self.Hessian-self.k*self.Phi.T@self.Hessian)
        self.Theta += self.mu*self.k*self.alpha
        self.kappa = np.linalg.norm(self.Hessian,np.inf)*np.linalg.norm(self.Phi,np.inf)
        
    def RLS_getInfo(self):
        return self.Hessian, self.Theta, self.kappa, self.y_est
    
    def RLS_calcEquivParams_Ren(self):
        self.Rs = -self.Theta[1]
        self.Rp = (self.Theta[0]*self.Theta[1]+self.Theta[2])/(self.Theta[1]-1)
        self.Cp = ((1-self.Theta[0])*self.Ts)/((self.Theta[0]*self.Theta[1]+self.Theta[2])*np.log(self.Theta[0]+1E-9)+1E-9)
        self.Beta = math.exp(-self.Ts/self.Rp/self.Cp)
        return self.Rs, self.Rp, self.Cp
    
    def RLS_EstimateOCV(self,Ut):
        numr = Ut-self.Beta*self.Phi[0]+self.Rs*self.Phi[1]+(self.Rp-self.Beta*(self.Rp+self.Rs))*self.Phi[2]
        self.OCVhat = numr/(1-self.Beta)
        return self.OCVhat
    
    def RLS_run(self,V_,I_):
        
        Rs = 0
        Rp = 0
        Cp = 0
        OCVhat = 0
        est_V = 0
        
        V = np.array(V_)
        I = np.array(I_)
        
        if self.count > 4:
            if self.EstMode == 0: # Differential measurements
               phi_U = (self.V_last-self.V_last_last).reshape(1,1)
               phi_I = -np.array([I-self.I_last,self.I_last-self.I_last_last]).reshape(2,1)
               self.RLS_update(phi_U, phi_I, V-self.V_last)
               
            elif self.EstMode == 1: # Direct measurements
               phi_U = (self.V_last).reshape(1,1)
               phi_I = -np.array([I,self.I_last]).reshape(2,1)
               self.RLS_update(phi_U, phi_I, V)
                
                
            P, theta, kappa, est_V = self.RLS_getInfo()
            Rs, Rp, Cp = self.RLS_calcEquivParams_Ren()
            OCVhat = self.RLS_EstimateOCV(V)
            
        else:
            self.count += 1
           
      # Assuming first-order RC with serial resistance, Rs = serial resistance,
      # Rp = parallel resistance, Cp = parallel capacitor
#        ParamVec = np.array([Rs,Rp,Cp])
           
           # Update measurements on exit
        self.V_last = V
        self.V_last_last = self.V_last
        self.I_last = V
        self.I_last_last = self.I_last
      
       
       # Return in following order: estimated parameter vector (Rs,Rp,Cp),
       # estimated OCV, estimated filter output
        return Rs, Rp, Cp, OCVhat, est_V
    
               
       
        
        
