%{
Copyright (c) 2017 Raghvendra V. Cowlagi. All rights reserved.

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
Simulations of pure pursuit and PN guidance schemes.
%}

clear variables; close all; clc;

%% Initial conditions
v0_target	= 5;				% m/s
yaw0_target	= 90*pi/180;		% rad
ptc0_target	= 0*pi/180;			% rad

target_maneuver_num = 1 + round(4*rand);
switch target_maneuver_num
	case {2, 4, 5}
		ptc_target0 = (-10+20*rand)*pi/180;
end

target_accel_lat = 0;
target_accel_vrt = 0;
switch target_maneuver_num
	case {3, 4}
		target_accel_lat	= 3*(-1 + 2*rand);
	case {5}
		target_accel_vrt	= 2*(-1 + 2*rand);
end

pos0_target	= [100; 5; -100];	% m

v0_missile	= 8;				% m/s
yaw0_missile= 0;				% rad
ptc0_missile= 0;				% rad
pos0_missile= [0; 0; -120];		% m




xi0	= [pos0_missile; v0_missile; yaw0_missile; ptc0_missile; ...
	pos0_target; v0_target; yaw0_target; ptc0_target];

%% Initial plot
figure('Name', 'Position', 'units', 'normalized', 'OuterPosition', ...
	[0.05 0.05 0.9 0.9]); hold on;  grid on; axis equal
draw_pv(pos0_missile, [v0_missile; yaw0_missile; ptc0_missile], '3D', 'b', 2)
draw_pv(pos0_target, [v0_target; yaw0_target; ptc0_target], '3D', 'r', 2)
view(3)
set(gca, 'ZDir', 'reverse')
xlabel('Position, x coordinate (m)')
ylabel('Position, y coordinate (m)')
zlabel('Position, z coordinate (m)')

%% Simulate strike scenario
time_span		= [0 200]; %0:0.01:10;
solver_options	= odeset('Events', @event_missilestrike_3D, 'RelTol', 1e-6);
[t_sim, xi_sim, t_events, state_event, idx_event] = ...
	ode45(@(t,xi) ode_engagement_3D(t,xi,target_maneuver_num,...
	target_accel_lat,target_accel_vrt), time_span, xi0, solver_options);

if numel(idx_event)
	switch idx_event
		case 1
			fprintf('Missile strike achieved!\n');
		case 2
			r_miss = norm(state_event(1:3) - state_event(7:9));
			fprintf('Simulation stopped due to negative closing velocity. \nMiss distance = %f m\n', r_miss);
	end			
end


%% Plot
figure('Name', 'Position', 'units', 'normalized', 'OuterPosition', ...
	[0.05 0.05 0.9 0.9]); hold on;  grid on; axis equal
draw_pv(pos0_missile, [v0_missile; yaw0_missile; ptc0_missile], '3D', 'b', 2)
draw_pv(pos0_target, [v0_target; yaw0_target; ptc0_target], '3D', 'r', 2)
view(3)
set(gca, 'ZDir', 'reverse')
xlabel('Position, x coordinate (m)')
ylabel('Position, y coordinate (m)')
zlabel('Position, z coordinate (m)')

pos_missile = xi_sim(:, 1:3);
v_missile	= xi_sim(:, 4);
yaw_missile = xi_sim(:, 5);
ptc_missile	= xi_sim(:, 6);

pos_target	= xi_sim(:, 7:9);
v_target	= xi_sim(:, 10);
yaw_target	= xi_sim(:, 11);
ptc_target	= xi_sim(:, 12);

for m1 = 1:numel(t_sim)	
	cla; hold on;
	draw_pv(pos_missile(m1, :), [v_missile(m1); yaw_missile(m1); ptc_missile(m1)], '3D', 'b', 2)
	draw_pv(pos_target(m1, :), [v_target(m1); yaw_target(m1); ptc_target(m1)], '3D', 'r', 2)
	plot3( pos_target(1:m1, 1), pos_target(1:m1, 2), pos_target(1:m1, 3), 'r')
	plot3( pos_missile(1:m1, 1), pos_missile(1:m1, 2), pos_missile(1:m1, 3), 'b')
	drawnow();
end

print_punch_line(pos_target, pos_missile);