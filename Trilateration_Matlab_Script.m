clear variables; close all; clc;

satellite_positions = [75 32 68; 90 0 60; 80 20 60]';
measured_ranges = [67.72; 80.07; 67.61];

receiver_position  = Trilateration_Matlab_Function(satellite_positions, measured_ranges)

