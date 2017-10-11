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
Kalman filter estimation algorithm for target seeker
%}

function [state_dot_seeker_est, state_dot_seeker_covar] = ...
	est_seeker(t, state_seeker_est, state_seeker_covar, seeker_meas, ...
	parameters_system)

if strcmp(parameters_system.simulation.use_seeker_ekf, 'no')
	state_dot_seeker_est	= zeros(6, 1);
	state_dot_seeker_covar	= zeros(36, 1);
	return;
end

C_skr	= eye(6);	% <-- ** INSERT RELATIONSHIP BETWEEN STATE AND MEASUREMENT

A_lin	= A_fun(state_seeker_est);

%***** INSERT LINEARIZED EQUATIONS OF MOTION (RELATIVE MOTION BETWEEN
%TARGET AND MISSILE) FOR A_lin. THE RELATIVE MOTION WILL DEPEND ON THE
%TARGET MANEVUER, AS WELL THE MISSILE PN GUIDANCE LAW.

%----- Computations of Kalman gain and estimation error covariance update
P_seeker	= reshape( state_seeker_covar, 6, 6);
Kalman_gain	= P_seeker * C_skr' * parameters_system.noise.Rinv_skr;

%%
% acceleration_cmd_inertial = pn_guidance_law_3d(state_seeker_est);

% state_dot_seeker_est = [state_seeker_est(4:6); acceleration_cmd_inertial] + ...
% 	Kalman_gain * (seeker_meas - C_skr*state_seeker_est);
%%

p_and_p_dot = state_seeker_est;

N1 = 50;
N2 = 50;

alpha_dot_fun = @(in1)(in1(1,:).*in1(5,:)-in1(4,:).*in1(2,:))./(in1(1,:).^2+in1(2,:).^2);
beta_dot_fun =  @(in1)1.0./sqrt((in1(1,:).^2+in1(2,:).^2)./(in1(1,:).^2+in1(2,:).^2+in1(3,:).^2)).*1.0./(in1(1,:).^2+in1(2,:).^2+in1(3,:).^2).^(3.0./2.0).*(in1(1,:).^2.*in1(6,:)+in1(2,:).^2.*in1(6,:)-in1(4,:).*in1(1,:).*in1(3,:)-in1(5,:).*in1(2,:).*in1(3,:));
v_c_fun = @(in1)-(in1(4,:).*in1(1,:)+in1(5,:).*in1(2,:)+in1(6,:).*in1(3,:)).*1.0./sqrt(in1(1,:).^2+in1(2,:).^2+in1(3,:).^2);

alpha = atan2(p_and_p_dot(2), p_and_p_dot(1));
% alpha = atan(p_and_p_dot(2) / p_and_p_dot(1));
beta = asin(p_and_p_dot(3) / norm(p_and_p_dot(1:3)));

alpha_dot = alpha_dot_fun(p_and_p_dot);
beta_dot = beta_dot_fun(p_and_p_dot);
v_c = v_c_fun(p_and_p_dot);

a_ch = N1 * v_c * alpha_dot;
a_cv = N2 * v_c * beta_dot;

%% f
los_1 = alpha;
los_2 = beta;
acv = a_cv;
ach = a_ch;

%%
state_dot_seeker_est= [state_seeker_est(4:6); ...
	ach*sin(los_1) + acv*cos(los_1)*sin(los_2); ...
	-ach*cos(los_1) + acv*sin(los_1)*sin(los_2); -acv*cos(los_2)] + ...
	Kalman_gain * (seeker_meas - C_skr*state_seeker_est);

state_dot_seeker_covar	= (A_lin - Kalman_gain*C_skr)*P_seeker + ...
	P_seeker*(A_lin - Kalman_gain*C_skr)' + ...
	Kalman_gain*parameters_system.noise.R_skr*Kalman_gain' + ...
	parameters_system.noise.Q_skr;
state_dot_seeker_covar	= state_dot_seeker_covar(:);