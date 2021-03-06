%% 2

sat_positions = ...
    [90 -70 95;
    -90 -5 60;
    -75 -20 85;
    60 80 -70;
    80 90 100;
    35 -45 55;
    -80 65 20]';

rho = ...
    [139.533;
    118.995;
    120.099;
    126.191;
    120.315;
    79.1232;
    110.215];
    
[receiver_position, epsilon_c] = trilat_clockbias(sat_positions, rho, zeros(3,1), 1e-3, 1e-3, 1e3)

