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
Ordinary differential equation that describes the strike scenario of a
missile attack on a target aircraft. Simulated elements 
include: missile and aircraft motion, missile navigation (IMU & GPS),
missile seeker and IMU sensors, missile guidance law, overall strike
scenario.

This function is called by the main script file ae4733_sim_missile_strike.m
%}

function state_dot_all = ode_missile_strike(t, state_all, parameters_system)

global input_missile_prev

%----- Readability: parameters
parameters_target	= parameters_system.target;
parameters_missile	= parameters_system.missile;

%----- Readability: function handles
fh_gps_aided_ins= parameters_system.simulation.fh_gps_ins;
fh_seeker		= parameters_system.simulation.fh_seeker;
fh_guidance		= parameters_system.simulation.fh_guidance;

%----- Readability: states
state_missile_true	= state_all(1:6);										% 3 posn + 3 velocity coordinates (true missile state)
state_target_true	= state_all(7:12);										% 3 posn + 3 velocity coordinates (true target state)
state_missile_est	= state_all(13:18);										% 3 posn + 3 velocity coordinates (estimated missile state)
state_seeker_est	= state_all(19:24);										% 3 posn + 3 velocity coordinates (estimated target state)
state_missile_covar	= state_all(25:60);										% 36 states of missile nav estimate error covariance matrix
state_seeker_covar	= state_all(61:96);										% 36 states of seeker estimate error covariance matrix

%----- Current time index
t_seq	= parameters_system.simulation.t_seq;
t_indx	= max( 2, find( (t_seq > t), 1, 'first'));
if ~numel(t_indx), t_indx = numel(t_seq) + 1; end

%----- Sensor measurements
[rates_meas, accel_meas]= sensors_inertial_strapdown(t_indx, ...
	state_missile_true, input_missile_prev, parameters_system);
gps_pv_meas	= sensor_gps(t_indx, state_missile_true, parameters_system);
seeker_meas	= sensor_seeker(t_indx, state_target_true, ...
	state_missile_true, parameters_system);

%----- Estimation
[state_dot_missile_est, state_dot_missile_covar]= ...
	fh_gps_aided_ins(t, state_missile_est, state_missile_covar, ...
	rates_meas, accel_meas, gps_pv_meas, parameters_system);

% if 
% 	[state_dot_seeker_est, state_dot_seeker_covar]	= ...
% 		fh_seeker(t, state_seeker_est, state_seeker_covar, seeker_meas, ...
% 		state_missile_true, state_dot_missile_true, parameters_system);
% else
% 	[state_dot_seeker_est, state_dot_seeker_covar]	= ...
% 		fh_seeker(t, state_seeker_est, state_seeker_covar, seeker_meas, ...
% 		state_missile_est, state_dot_missile_est, parameters_system);
% end

%----- Missile guidance
if strcmp(parameters_system.simulation.imu_noise, 'no') || ...
	strcmp(parameters_system.simulation.use_gps_ins_ekf, 'no')
	state_missile_est	= state_missile_true;
end
if strcmp(parameters_system.simulation.seeker_noise, 'no') || ...
	strcmp(parameters_system.simulation.use_seeker_ekf, 'no')
	state_seeker_est	= seeker_meas;
end

input_missile		= fh_guidance(state_missile_est, state_seeker_est);
input_missile_prev	= input_missile;
%----- Target maneuver
input_target		= calc_target_inputs(t, parameters_target);

%----- True system states
state_dot_missile_true	= ode_3Dparticle(t, state_missile_true, ...
	input_missile, parameters_missile);
state_dot_target_true	= ode_3Dparticle(t, state_target_true, ...
	input_target, parameters_target);

if strcmp(parameters_system.simulation.use_gps_ins_ekf, 'no')
	[state_dot_seeker_est, state_dot_seeker_covar]	= ...
		fh_seeker(t, state_seeker_est, state_seeker_covar, seeker_meas, ...
		parameters_system);
else
	[state_dot_seeker_est, state_dot_seeker_covar]	= ...
		fh_seeker(t, state_seeker_est, state_seeker_covar, seeker_meas, ...
		parameters_system);
end

state_dot_all	= [state_dot_missile_true; state_dot_target_true; ...
	state_dot_missile_est; state_dot_seeker_est; ...
	state_dot_missile_covar; state_dot_seeker_covar];
