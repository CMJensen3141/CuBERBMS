# -*- coding: utf-8 -*-
"""
Created on Tue Aug 30 12:55:22 2022

@author: Christian
"""

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("TestData/BMS_LAB_TEST_REFSTEPS_2.csv")

df.plot(subplots = True, y = ["Reg_Ctrl_BMS","Reg_Stack_voltage","Reg_Stack_Current","Reg_P_ref","Reg_Load_Ref","Reg_Flow_Anolyte","Reg_AC_DC_GRID_POWER"])
plt.show()
