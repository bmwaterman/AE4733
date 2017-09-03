clear variables; close all; clc;

satellite_positions = [90 -70 95; -90 -5 60 ; -75 -20 85;60 80 -70 ]';
measured_ranges = [139.533; 118.995; 120.099; 126.191];

[receiver_position, epsilon_c]  = Trilateration_Matlab_Function_Clockbias(satellite_positions, measured_ranges)

