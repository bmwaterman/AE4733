%{
Copyright (c) 2014 Raghvendra V. Cowlagi. All rights reserved.

Copyright notice: 
=================
No part of this work may be reproduced without the written permission of
the copyright holder, except for non-profit and educational purposes under
the provisions of Title 17, USC Section 107 of the United States Copyright
Act of 1976. Reproduction of this work for commercial use is a violation of
copyright.


Disclaimer:
===========
This software program is intended for educational and research purposes.
The author and the institution with which the author is affiliated are not
liable for damages resulting the application of this program, or any
section thereof, which may be attributed to any errors that may exist in
this program.


Author information:
===================
Raghvendra V. Cowlagi, Ph.D,
Assistant Professor, Aerospace Engineering Program,
Department of Mechanical Engineering, Worcester Polytechnic Institute.
 
Higgins Laboratories, 247,
100 Institute Road, Worcester, MA 01609.
Phone: +1-508-831-6405
Email: rvcowlagi@wpi.edu
Website: http://www.wpi.edu/~rvcowlagi


The author welcomes questions, comments, suggestions for improvements, and
reports of errors in this program.


Program description:
====================
Simulation of a missile attack on a target aircraft. Simulated elements
include: missile and aircraft motion, missile navigation (IMU & GPS),
missile seeker and IMU sensors, missile guidance law, overall strike
scenario.
%}

clear variables; close all; clc

global input_missile_prev

load dramatic_music.mat
sound(y,Fs);				% <-- ** COMMENT THIS LINE IF YOU DON'T WANT MUSIC

%% Setup simulation parameters

seeker_noise	= 'yes';	% <-- ** CHANGE TO 'yes' TO INTRODUCE SEEKER NOISE
use_seeker_ekf	= 'yes';	% <-- ** CHANGE TO 'yes' TO SEE IF YOUR SEEKER EKF WORKS

target_maneuver_num	= 1;	% <-- ** CHANGE TARGET MANEUVER NUMBER HERE

[parameters_system, initial_conditions] = ...
	setup_system_parameters(target_maneuver_num, seeker_noise, use_seeker_ekf);

parameters_system.simulation.fh_seeker		= @est_seeker;					% <-- ** COMPLETE THE CODE IN THE FILE est_seeker.m
parameters_system.simulation.fh_guidance	= @pn_3D_placeholder;			% <-- ** TYPE IN THE NAME OF YOUR GUIDANCE FUNCTION HERE


%% Setup simulation

input_missile_prev	= initial_conditions.input_missile_prev;
pos_missile_init	= initial_conditions.pos_missile_init;
pos_target_init		= initial_conditions.pos_target_init;
spd_missile_init	= initial_conditions.spd_missile_init;
spd_target_init		= initial_conditions.spd_target_init;
yaw_missile_init	= initial_conditions.yaw_missile_init;
yaw_target_init		= initial_conditions.yaw_target_init;
ptc_missile_init	= initial_conditions.ptc_missile_init;
ptc_target_init		= initial_conditions.ptc_target_init;

gps_pv_meas			= sensor_gps(2, [pos_missile_init; spd_missile_init; ...
	yaw_missile_init; ptc_missile_init], parameters_system);
state_missile_est_init  = [gps_pv_meas(1:3); spd_missile_init + 5*randn; 0.1*randn; 0.1*randn];

state_seeker_est_init	= [pos_target_init; spd_target_init*...
	[cos(yaw_target_init)*cos(ptc_target_init); ...
	sin(yaw_target_init)*cos(ptc_target_init); -sin(ptc_target_init)] ] - ...
	[pos_missile_init; spd_missile_init*...
	[cos(yaw_missile_init)*cos(ptc_missile_init); ...
	sin(yaw_missile_init)*cos(ptc_missile_init); -sin(ptc_missile_init)] ] + ...
	parameters_system.simulation.noise_skr(:, 1);

state_missile_covar_init= reshape(eye(6), 36, 1);
state_seeker_covar_init	= reshape(parameters_system.noise.R_skr, 36, 1);

state_all_init = [...
	pos_missile_init; spd_missile_init; yaw_missile_init; ptc_missile_init; ...
	pos_target_init; spd_target_init; yaw_target_init; ptc_target_init; ...
	state_missile_est_init; state_seeker_est_init; ...
	state_missile_covar_init; state_seeker_covar_init];

%% Simulation of overall strike scenario
solver_options	= ...
	odeset('Events', @(t,state_all) ...
	parameters_system.simulation.fh_strike_event(t, state_all, parameters_system), ...
	'RelTol', 1e-4);
[t_sim, state_all_sim, t_events, state_event, idx_event] = ode45(@(t, state_all) ...
	ode_missile_strike(t, state_all, parameters_system), ...
	parameters_system.simulation.t_seq, state_all_init, solver_options);

if numel(idx_event)
	switch idx_event
		case 1
			fprintf('Missile strike achieved!\n');
		case 2
			r_miss = norm(state_event(1:3) - state_event(7:9));
			fprintf('Simulation stopped due to negative closing velocity. \nMiss distance = %f m\n', r_miss);
	end			
end

%% Result plots and animation
figure('Name', 'Position', 'units', 'normalized', 'OuterPosition', ...
	[0.05 0.05 0.9 0.9]); hold on;  grid on; axis equal
draw_pv(pos_missile_init, [spd_missile_init; yaw_missile_init; ptc_missile_init], '3D', 'b', 2)
draw_pv(pos_target_init, [spd_target_init; yaw_target_init; ptc_target_init], '3D', 'r', 2)
view(3)
set(gca, 'ZDir', 'reverse')
xlabel('Position, x coordinate (m)')
ylabel('Position, y coordinate (m)')
zlabel('Position, z coordinate (m)')

pos_missile = state_all_sim(:, 1:3);
spd_missile	= state_all_sim(:, 4);
yaw_missile = state_all_sim(:, 5);
ptc_missile	= state_all_sim(:, 6);

pos_target	= state_all_sim(:, 7:9);
spd_target	= state_all_sim(:, 10);
yaw_target	= state_all_sim(:, 11);
ptc_target	= state_all_sim(:, 12);

for m1 = 1:ceil(numel(t_sim)/150):numel(t_sim)	
	cla; hold on;
	draw_pv(pos_missile(m1, :), [spd_missile(m1); yaw_missile(m1); ptc_missile(m1)], '3D', 'b', 2)
	draw_pv(pos_target(m1, :), [spd_target(m1); yaw_target(m1); ptc_target(m1)], '3D', 'r', 2)
	plot3( pos_target(1:m1, 1), pos_target(1:m1, 2), pos_target(1:m1, 3), 'r')
	plot3( pos_missile(1:m1, 1), pos_missile(1:m1, 2), pos_missile(1:m1, 3), 'b')
	drawnow();
end

print_punchline(pos_missile, pos_target, parameters_system)

pause(10)
clear sound
