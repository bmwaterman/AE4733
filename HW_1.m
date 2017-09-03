%%% GNC Homework 1
% Bailey Waterman, Keshuai Xu

%% 1a
% We set up the axes that y is due north and the x-y plane is tangent to
% the earth surface. Turning 90 degrees in x-y plane will result in *almost* East heading.
% We ignore the curvature of the earth since the distance between Worcester
% and Boston is relatively small.

TAS = 70; %m/s
g = 9.8039; %gravitational constant at Boston (m/s^2)
bank = 20*pi/180; %radians
syms r;
% Angle of bank formula
% http://www.luizmonteiro.com/Article_Bank_Angle_for_Std_Rate_01.aspx
turn_radius = vpasolve(bank-atan(((TAS^2)/r)/g),r,1000); % m
time = vpa(((90*pi/180)*turn_radius)/TAS, 6) % s

%% 1b
% ignoring the curvature of the earth
%
%
% <<q1a.jpg>>
%

g_rounded = 10; %(m/s^2)
turn_radius_rounded = vpasolve(bank-atan(((TAS^2)/r)/g_rounded),r,1000); % m
time_rounded = round(((90*pi/180)*turn_radius_rounded)/TAS); % s

omega_rounded = TAS/turn_radius_rounded; % rad/s
theta_rounded = time_rounded*omega_rounded; % rad
P_rounded = [turn_radius_rounded - turn_radius_rounded * cos(theta_rounded); % x in m
    turn_radius_rounded * sin(theta_rounded); % y in m
    pi/2 - theta_rounded]; % heading in rad wrt x axis
P = [turn_radius; turn_radius; 0]; % [m; m; rad]
error = P_rounded - P; % error in [x, y, heading]. unit in [m; m; rad]
vpa(error, 6)

%% 1c
%
% <<q1b.jpg>>
%

P_rounded = [turn_radius_rounded - turn_radius_rounded * cos(theta_rounded); % x in m
    turn_radius_rounded * sin(theta_rounded) + TAS*5; % y in m
    pi/2 - theta_rounded]; % heading in rad wrt x axis
P = [turn_radius; turn_radius; 0]; % [m; m; rad]
error = P_rounded - P; % error in [x, y, heading]. unit in [m; m; rad]
vpa(error, 6)


%% 2a
%
% <<q2a.jpg>>
%
% <<q2a2.jpg>>
%

%% 2b
% Result:
%
% <<q2b.jpg>>
%
% Trilateration function:
%
% <include>trilat_clockbias.m</include>
% 

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
    
[receiver_position, epsilon_c] = trilat_clockbias(sat_positions, rho, zeros(3,1), 1e-6, 1e-6, 1e3)

