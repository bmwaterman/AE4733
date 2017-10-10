%{
Copyright (c) 2017 Raghvendra V. Cowlagi. All rights reserved.

Copyright notice: 
=================
No part of this work may be reproduced without the written permission of
the copyright holder, except for non-profit and educational purposes under
the provisions of Title 17, USC Section 107 of the United States Copyright
Act of 1976. Reproduction of this work for commercial use is a violation of
copyright.

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

Program description:
====================
Simulations of pure pursuit and PN guidance schemes.
%}

clear all; close all; clc;

%% Initial conditions
v0_target	= 5;		% m/s
yaw0_target	= pi/2;		% rad
pos0_target	= [100; 5];	% m

v0_missile	= 60;		% m/s
yaw0_missile= -60*pi/180;		% rad
pos0_missile= [0; 0];	% m

xi0	= [pos0_missile; v0_missile; yaw0_missile; pos0_target; v0_target; yaw0_target];

%% Initial plot
figure('Name', 'Position', 'units', 'normalized', 'OuterPosition', ...
	[0 0 0.95 0.95]); hold on;  grid on; axis equal
plot(pos0_missile(1), pos0_missile(2), 'bo', 'MarkerFaceColor', 'b');
plot(pos0_target(1), pos0_target(2), 'ro', 'MarkerFaceColor', 'r');
quiver(pos0_target(1), pos0_target(2), v0_target*cos(yaw0_target), ...
	v0_target*sin(yaw0_target), 'LineWidth', 2, 'Color', 'r', 'MaxHeadSize', 10)
quiver(pos0_missile(1), pos0_missile(2), v0_missile*cos(yaw0_missile), ...
	v0_missile*sin(yaw0_missile), 'LineWidth', 2, 'Color', 'b', 'MaxHeadSize', 10)
%xlim([-50 200]); ylim([-50 200]);
xlabel('Position, x coordinate (m)')
ylabel('Position, y coordinate (m)')

%% Simulate strike scenario
time_span		= [0 100]; %0:0.01:10;
solver_options	= odeset('Events', @event_missilestrike_2D, 'RelTol', 1e-6);
[t_sim, xi_sim] = ode45(@ode_engagement_2D, time_span, xi0, solver_options);

%% Plot
pos_missile = xi_sim(:, 1:2);
v_missile	= xi_sim(:, 3);
yaw_missile = xi_sim(:, 4);

pos_target	= xi_sim(:, 5:6);
v_target	= xi_sim(:, 7);
yaw_target	= xi_sim(:, 8);

r_miss_sim		= zeros(numel(t_sim), 1);
v_closing_sim	= zeros(numel(t_sim), 1);
los_angle_sim	= zeros(numel(t_sim), 1);
los_rate_sim	= zeros(numel(t_sim), 1);

for m1 = 1:numel(t_sim)
	xdot_missile= v_missile(m1)*cos(yaw_missile(m1));
	ydot_missile= v_missile(m1)*sin(yaw_missile(m1));
	xdot_target = v_target(m1)*cos(yaw_target(m1));
	ydot_target = v_target(m1)*sin(yaw_target(m1));

	r_miss_sim(m1)		= norm(pos_target(m1, :) - pos_missile(m1, :));
	
	v_closing_sim(m1)	= -( (pos_target(m1,1) - pos_missile(m1,1))*(xdot_target - xdot_missile) + ...
		(pos_target(m1,2) - pos_missile(m1,2))*(ydot_target - ydot_missile) ) / r_miss_sim(m1);

	
	los_angle_sim(m1)	= atan2( (pos_target(m1,2) - pos_missile(m1,2)), (pos_target(m1,1) - pos_missile(m1,1)) );
	los_rate_sim(m1)	= ( (pos_target(m1,1) - pos_missile(m1,1))/( (r_miss_sim(m1)^3)*cos(los_angle_sim(m1))) ) * ...
		( (ydot_target - ydot_missile)*(pos_target(m1,1) - pos_missile(m1,1)) - ...
		(pos_target(m1,2) - pos_missile(m1,2))*(xdot_target - xdot_missile) );
	
% 	pn_constant = 3;
% 	fn_missile_max = 3;
% 	pn_guidance_command = pn_constant * v_closing_sim(m1) * los_rate_sim(m1);
% 	pn_guidance_command = sign(pn_guidance_command)*min( abs(pn_guidance_command), fn_missile_max );
	
	
	cla; hold on;
	plot(pos_missile(m1,1), pos_missile(m1,2), 'bo', 'MarkerFaceColor', 'b');
	plot(pos_target(m1,1), pos_target(m1,2), 'ro', 'MarkerFaceColor', 'r');
	quiver(pos_target(m1,1), pos_target(m1,2), v_target(m1)*cos(yaw_target(m1)), ...
		v_target(m1)*sin(yaw_target(m1)), 'LineWidth', 2, 'Color', 'r', 'MaxHeadSize', 10);
	quiver(pos_missile(m1,1), pos_missile(m1,2), v_missile(m1)*cos(yaw_missile(m1)), ...
		v_missile(m1)*sin(yaw_missile(m1)), 'LineWidth', 2, 'Color', 'b', 'MaxHeadSize', 10);
	plot( pos_missile(1:m1, 1), pos_missile(1:m1, 2), 'b')
	plot( pos_target(1:m1, 1), pos_target(1:m1, 2), 'r')
	drawnow();
end
% text(pos_target(end, 1)- 15, pos_target(end, 2) - 3, 'That''s a Bingo!', 'FontSize', 20, 'FontWeight', 'bold');

print_punch_line_2D(pos_target, pos_missile);

% 
figure;
subplot(4, 1,1)
plot(t_sim, r_miss_sim, 'LineWidth', 2); grid on;
ylabel('Miss distance (m)')

subplot(4, 1,2)
plot(t_sim, v_closing_sim, 'LineWidth', 2); grid on;
ylabel('Closing velocity (m/s)')

subplot(4, 1,3) 
plot(t_sim, los_angle_sim*180/pi, 'LineWidth', 2); grid on;
ylabel('LOS angle (deg)')

subplot(4, 1,4)
plot(t_sim, los_rate_sim*180/pi, 'LineWidth', 2); grid on;
ylabel('LOS rate (deg/s)')

