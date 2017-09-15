clear all; close all; clc;

%initial conditions
euler321_angles_initial = [10; 15; 13] * pi/180; %radians
time_span = [0 2]; %seconds
omega_iee = [2; 0; 0;] *pi/180; %radians/second

%solve ODE
[t_sim, euler321_angles_sim] = ...
ode45(@(t, y)rotational_kinematics(t, y, omega_iee), time_span, euler321_angles_initial);

%initial conditions same as last ones
euler321_angles_initial = euler321_angles_sim(end, :); %rad
time_span = [2 5]; %s
omega_iee = [1; 3; 0;] * pi/180; %rad/s

%solve ODE
[t_sim_2, euler321_angles_sim_2] = ...
ode45(@(t, y)rotational_kinematics(t, y, omega_iee), time_span, euler321_angles_initial);
    
%initial conditions same as last ones
euler321_angles_initial = euler321_angles_sim_2(end, :); %rad
time_span = [5 8]; %s
omega_iee = [0; 0; 1;] * pi/180; %rad/s    

%solve ODE
[t_sim_3, euler321_angles_sim_3] = ...
ode45(@(t, y)rotational_kinematics(t, y, omega_iee), time_span, euler321_angles_initial);

%initial conditions same as last ones
euler321_angles_initial = euler321_angles_sim_3(end, :); %rad
time_span = [8 10]; %s
omega_iee = [1; 4; 3;] * pi/180; %rad/s    

%solve ODE
[t_sim_4, euler321_angles_sim_4] = ...
ode45(@(t, y)rotational_kinematics(t, y, omega_iee), time_span, euler321_angles_initial);

euler321_angles_degrees_1 = euler321_angles_sim * 180/pi;
euler321_angles_degrees_2 = euler321_angles_sim_2 * 180/pi;
euler321_angles_degrees_3 = euler321_angles_sim_3 * 180/pi;
euler321_angles_degrees_4 = euler321_angles_sim_4 * 180/pi;

time = [t_sim; t_sim_2; t_sim_3; t_sim_4;];
euler321_angles_degrees = [euler321_angles_degrees_1; euler321_angles_degrees_2; euler321_angles_degrees_3; euler321_angles_degrees_4;];   

plot(time, euler321_angles_degrees)

xlabel('time(sec)')
ylabel ('angle(deg)')
legend ('yaw(psi)','pitch(theta)','roll(phi)')


