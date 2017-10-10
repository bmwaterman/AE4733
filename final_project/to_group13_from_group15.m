clear all; close all; clc; 

N = 2500;
lookup_time = linspace(0, 1000, N);
lookup_accel= [7* ones(1, 10), 9.81* ones(1, 75), 11 * ones(1, 250), zeros(1, 300), 3 * ones(1, 365), -17* ones(1,700), -6 * ones(1, 300), -12 * ones(1, 200), 7 * ones(1, 300);
    30 * ones(1, 30), 5.7 * ones(1, 175), ones(1, 350), -10 * ones(1, 425), 12 * ones(1, 300), 17 * ones(1, 200), 21 * ones(1, 500), 40 * ones(1, 180), -6 * ones(1, 340);
    -4.2 * ones(1, 100), 3.8 * ones(1, 200), 7 * ones(1, 300), 11 * ones(1, 400), 15* ones (500), 5 * ones(600), -13 * ones(1, 400)];
          


lookup_time

lookup_accel
