%% homework 3
% 
clear variables, clc, close all
%% Problem 4
p4_data = load('hw3_p4.mat');
y0 = [p4_data.ptt_0; p4_data.vtt_0; p4_data.yaw_0; p4_data.ptc_0; p4_data.rol_0];
[t_sim, y_sim] = ode45(@(t,y) ins_fun(t,y,p4_data), p4_data.time_stamp, y0);

figure();
plot3(y_sim(:,1),y_sim(:,2),y_sim(:,3));
title('position trajectory')

figure();
plot(t_sim, y_sim(:,4:6));
xlabel('time (sec)');
ylabel ('velocity (m/s)');
title('velocity');

figure();
plot(t_sim, rad2deg(y_sim(:,6:9)));
xlabel('time (sec)');
ylabel ('euler angles (deg)');
title('euler angles');
legend ('yaw(psi)','pitch(theta)','roll(phi)')

%% problem 6
p6_data = load ('hw3_p6.mat');
% p6_temperature = p6_data.measurements(1:1500);
% p6_angle_deg = p6_data(1501:3000);
% p6_range = p6_data(3001:5000);

t0 = 288.16; % Kelvin
a1 = -6.5e-3; % Kelvin/meter

N_temp = normrnd(0, 2, 1500,1);    % Normrnd function creates random numbers within the standard deviation
N_angle = normrnd(0, 0.5, 1500, 1); % I made a 1500x1 matrix of random #s for temperature, a 1500x1 matrix with random #s 
N_range = normrnd(0, 100, 2000, 1); % for angle and a 200x1 matrix with random #s for range

N = [N_temp; N_angle; N_range;]; %I combined these three noise readings to form a single matrix

offset = [- t0 * ones(1500, 1); zeros(3500,1)];
z = (p6_data.measurements + offset) - N;                                % I subtracted the noise from the measurements %% this is the part I'm not sure about beacause
C = [ones(1500,1) * a1; ones(1500,1) * 180 / (pi * 1e4); ones(2000,1)]; % I'm not sure adjusting the equation he gave us in class like this will still result in 
xi = C \ z                                                              % the least squares estimate formula

%% problem 7.1
clear variables
data = csvread('hw3q7.csv',1,0);
plot(data(:, 1),data(:, 2:end))

%% problem 7.2

y0 = [zeros(3,1);zeros(3,1); zeros(3,1)];
p7_data.time_stamp = data(:, 1)';
p7_data.accel_readings = data(:, 2:4)';
p7_data.gyro_readings = data(:, 7:-1:5)'; % csv data is roll pitch yaw
[t_sim, y_sim] = ode45(@(t,y) ins_fun(t,y,p7_data), p7_data.time_stamp, y0);

figure();
plot3(y_sim(:,1),y_sim(:,2),y_sim(:,3));
title('position trajectory')

figure();
plot(t_sim, y_sim(:,4:6));
xlabel('time (sec)');
ylabel ('velocity (m/s)');
title('velocity');

figure();
plot(t_sim, rad2deg(y_sim(:,6:9)));
xlabel('time (sec)');
ylabel ('euler angles (deg)');
title('euler angles');
legend ('yaw(psi)','pitch(theta)','roll(phi)')

