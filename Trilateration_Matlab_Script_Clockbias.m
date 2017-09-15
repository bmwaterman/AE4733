clear variables; close all; clc;

satellite_positions = [35 -45 55; -80 65 20 ;-75,-20 85 ;60 80 -70 ]';
measured_ranges = [79.1232; 110.215; 120.099; 126.191];

[receiver_position, epsilon_c]  = Trilateration_Matlab_Function_Clockbias(satellite_positions, measured_ranges)

