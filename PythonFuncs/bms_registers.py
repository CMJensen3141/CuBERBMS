# -*- coding: utf-8 -*-
"""
Created on Thu Aug 18 13:27:16 2022

@author: chris
"""

Reg_Ctrl_BMS 				= 100; #Control of the battery (charge or discharge)
Reg_SOC						= 101; #State of charge of the battery
Reg_P_ref					= 102; #Reference for charge level (positive = discharging from battery to grid)
Reg_GaussVolt				= 103; #Gaussian charge voltage on surface of tank
Reg_SOC_Gauss				= 104; #State of charge estimation based on Gaussian charge voltage
Reg_BMS_Error				= 105; #Errors from BMS (0= no error)
Reg_Stack_Current			= 106; #Current in the stack of the CuRFB
Reg_Stack_voltage			= 107; #Voltage in the stack of the CuRFB
Reg_Chlorine				= 108; #Formation of Chlorine due to overvoltage
Reg_Temp_anolyte			= 109; #Temperature in the anolyte tank
Reg_Temp_anolyte_In			= 110; #Temperature at the entrance of the stack
Reg_Flow_Anolyte			= 111; #Flow of the anolyte circuit
Reg_Pressure_Anolyte		= 112; #Pressure in the anolyte circuit
Reg_Speed_Anolyte			= 113; #Pump speed for the anolyte circuit
Reg_Temp_catholyte			= 114; #Temperature in the catholyte tank
Reg_Temp_catholyte_In		= 115; #Temperature at the entrance of the stack
Reg_Flow_catholyte			= 116; #Flow of the catholyte circuit
Reg_Pressure_catholyte		= 117; #Pressure in the catholyte circuit
Reg_Speed_catholyte			= 118; #Pump for the catholyte circuit
Reg_DC_DC_ON				= 119; #DC/DC converter (on/off)
Reg_DC_DC_CURRENT			= 120; #Charging/discharging current in DC/DC converter
Reg_DC_DC_VOLTAGE			= 121; #Charging voltage/cell voltage in DC/DC converter
Reg_DC_DC_POWER				= 122; #Charging power in DC/DC converter (positive = discharing)
Reg_DC_DC_TEMP				= 123; #Internal temperature in DC/DC converter
Reg_DC_DC_AMBIENT_TEMP		= 124; #Ambient temperature in DC/DC converter
Reg_DC_DC_ERRORS			= 125; #Errors from DC/DC converter (0 = no error)
Reg_AC_DC_ON				= 126; #AC/DC converter (on/off)
Reg_AC_DC_GRID_CURRENT		= 127; #AC/DC converter current (positive = discharging)
Reg_AC_DC_GRID_VOLTAGE		= 128; #Grid voltage in AC/DC converter (phase to phase)
Reg_AC_DC_GRID_POWER		= 129; #Grid power in AC/DC converter (positive = output)
Reg_DC_LINK_VOLTAGE			= 130; #DC link voltage between AC/DC and DC/DC converters
Reg_AC_DC_TEMP				= 131; #Internal temperature in AC/DC converter
Reg_AC_DC_AMBIENT_TEMP		= 132; #Ambient temperature in AC/DC converter
Reg_AC_DC_ERRORS			= 133; #Errors from AC/DC converter (0 = no error)
Reg_Speed_Thermal			= 134; #Pump speed for thermal circuit
Reg_T_Thermal_tank			= 135; #Temperature in the thermal tank
Reg_Simulation_mode			= 136; #0=Live version on real HW, 1=Simulation level 1 (forward Euler, 1. order)
Reg_Time_contraction		= 137; #Multiplier for time in simulation (speed up factor), reduces accuracy of simulation
Reg_Simulation_clock		= 138; #Internal simulation in seconds
Reg_thermal_temp_catholyte  = 139; #Temperature of the heat exchanger in the catholyte circuit
Reg_thermal_temp_anolyte    = 140; #Temperature of the heat exchanger in the anolyte circuit
Reg_anolyte_pressure_out    = 141; #Pressure in the anolyte circuit after the stack
Reg_catholyte_pressure_out  = 142; #Pressure in the catholyte circuit after the stack
Reg_anolyte_temp_out        = 143; #Temperature in the anolyte circuit after the stack
Reg_catholyte_temp_out      = 144; #catholyte_temp_out
Reg_rs_est                  = 145; # serial resistance * 1000 
Reg_rp_est                  = 146; 
Reg_cp_hi_est               = 147; # cp >> 15
Reg_cp_lo_est               = 148; # cp & ox7fff
Reg_ocv_est                 = 149; # ocv * 100

HOLDING_REGISTERS           = 139;
HOLDING_REGISTER_START      = 99;
HOLDING_REGISTER_END        = 149;
HOLDING_REGISTER_MAX        = HOLDING_REGISTER_START + HOLDING_REGISTERS ;
HOLDING_REGISTER_COUNT      = HOLDING_REGISTER_END - HOLDING_REGISTER_START -1;
# HOLDING_REGISTER_COUNT = 44;


INVERTER_STANDBY = 0
INVERTER_CHARGING = 1 #from grid to battery
INVERTER_DISCHARGING = -1 #from battery to grid

BATTERY_STANDBY = 0
BATTERY_CHARGING = 1 #from grid to battery
BATTERY_DISCHARGING = -1 #from battery to grid
BATTERY_UNDEFINED = 42 #from battery to grid

BMS_STANDBY = 0
BMS_CHARGING = 1
BMS_DISCHARGING = 2

RegNames = ['Reg_Ctrl_BMS',
            'Reg_SOC',
            'Reg_P_ref',
            'Reg_GaussVolt',
            'Reg_SOC_Gauss',
            'Reg_BMS_Error',
            'Reg_Stack_Current',
            'Reg_Stack_voltage',
            'Reg_Chlorine',
            'Reg_Temp_anolyte',
            'Reg_Temp_anolyte_In',
            'Reg_Flow_Anolyte',
            'Reg_Pressure_Anolyte',
            'Reg_Speed_Anolyte',
            'Reg_Temp_catholyte',
            'Reg_Temp_catholyte_In',
            'Reg_Flow_catholyte',
            'Reg_Pressure_catholyte',
            'Reg_Speed_catholyte',
            'Reg_DC_DC_ON',
            'Reg_DC_DC_CURRENT',
            'Reg_DC_DC_VOLTAGE',
            'Reg_DC_DC_POWER',
            'Reg_DC_DC_TEMP',
            'Reg_DC_DC_AMBIENT_TEMP',
            'Reg_DC_DC_ERRORS',
            'Reg_AC_DC_ON',
            'Reg_AC_DC_GRID_CURRENT',
            'Reg_AC_DC_GRID_VOLTAGE',
            'Reg_AC_DC_GRID_POWER',
            'Reg_DC_LINK_VOLTAGE',
            'Reg_AC_DC_TEMP',
            'Reg_AC_DC_AMBIENT_TEMP',
            'Reg_AC_DC_ERRORS',
            'Reg_Speed_Thermal',
            'Reg_T_Thermal_tank',
            'Reg_Simulation_mode',
            'Reg_Time_contraction',
            'Reg_Simulation_clock',
            'Reg_thermal_temp_catholyte',
            'Reg_thermal_temp_anolyte',
            'Reg_anolyte_pressure_out',
            'Reg_catholyte_pressure_out',
            'Reg_anolyte_temp_out',
            'Reg_catholyte_temp_out'
            'Reg_rs_est',
            'Reg_rp_est',
            'Reg_cp_hi_est',
            'Reg_cp_lo_est',
            'Reg_ocv_est']