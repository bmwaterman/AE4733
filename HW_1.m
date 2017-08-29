%% 1a
TAS = 70; %m/s
g = 9.8039; %gravitational constant at Boston (m/s^2)
bank = 20*pi/180; %radians
syms r;
turn_radius = vpasolve(bank-atan(((TAS^2)/r)/g),r,1000); % m
time = vpa(((90*pi/180)*turn_radius)/TAS) % s

%% 1b
%ignoring the curvature of the earth
g_rounded = 10; %(m/s^2)
turn_radius_rounded = vpasolve(bank-atan(((TAS^2)/r)/g_rounded),r,1000); % m
time_rounded = round(((90*pi/180)*turn_radius_rounded)/TAS); % s

omega_rounded = TAS/turn_radius_rounded; % rad/s technically should be negative
theta_rounded = time_rounded*omega_rounded % rad
P_rounded = [turn_radius_rounded - turn_radius_rounded * cos(theta_rounded); % x in m
    turn_radius_rounded * sin(theta_rounded); % y in m
    pi/2 - theta_rounded]; % heading in rad wrt x axis
P = [turn_radius; turn_radius; 0];
error = P_rounded - P; % [m; m; rad]
vpa(error)

%% 1c
P_rounded = [turn_radius_rounded - turn_radius_rounded * cos(theta_rounded); % x in m
    turn_radius_rounded * sin(theta_rounded) + TAS*5; % y in m
    pi/2 - theta_rounded]; % heading in rad wrt x axis
P = [turn_radius; turn_radius; 0];
error = P_rounded - P; % [m; m; rad]
vpa(error)


