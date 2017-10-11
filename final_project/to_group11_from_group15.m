clear all; close all; clc; 

N = 5000;
lookup_time = linspace(0, 1000, N);
lookup_accel= [0.5* ones(1, 10), 9.81* ones(1, 75), 3.1 * ones(1, 250), zeros(1, 300), ...
    4 * ones(1, 365), 3.8 * ones(1, 400), 3 * ones(1, 300), 2.5 * ones(1, 300), 2 * ones(1, 1000), 1.2 * ones(1, 400), 3 * ones(1, 700), ...
    1.5 * ones(1, 300), 5 * ones(1, 200), 1.7 * ones(1, 400); 
    2 * ones(1, 30), 5.7 * ones(1, 175), 4.5 * ones(1, 350), 1 * ones(1, 425), 2 * ones(1, 20), ...
    3 * ones(1, 100), 4 * ones(1, 600), 5 * ones(1, 300), 6 * ones(1, 700), 7 * ones(1, 400), ...
    8 * ones(1, 80), 9 * ones(1, 120), 2 * ones(1, 300), zeros(1, 1400);
    4.2 * ones(1, 100), 3.8 * ones(1, 200), 7 * ones(1, 300), 1.1 * ones(1, 400), 7 * ones(1, 500), 3.8 * ones(1, 600), ...
    4.2 * ones(1, 700), 3.8 * ones(1, 800), 7 * ones(1, 900), 5 * ones(1, 500)];
          


lookup_time

lookup_accel



save('to_group11_from_group15.mat')
save('to_group13_from_group15.mat')
save('to_group17_from_group15.mat')
