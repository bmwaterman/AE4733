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
	est_seeker2(t, state_seeker_est, state_seeker_covar, seeker_meas, ...
	parameters_system)

if strcmp(parameters_system.simulation.use_seeker_ekf, 'no')
	state_dot_seeker_est	= zeros(6, 1);
	state_dot_seeker_covar	= zeros(36, 1);
	return;
end

C_skr   = eye(6);   % <-- ** INSERT RELATIONSHIP BETWEEN STATE AND         MEASUREMENT

% 57
rx = state_seeker_est(1);
ry = state_seeker_est(2);
rz = state_seeker_est(3);

rx_dot = state_seeker_est(4);
ry_dot = state_seeker_est(5);
rz_dot = state_seeker_est(6);

% 69

rh = sqrt(rx^2 + ry^2);
r = sqrt(rx^2 + ry^2 + rz^2);

v_cl = -(rx*rx_dot + ry*ry_dot + rz*rz_dot)/r;

% 78
los_1 = atan2(ry, rx);
los_2 = asin(rz / r);
los_1_rate = (ry_dot*rx - rx_dot*ry)/(rh^2);
los_2_rate = (rz_dot*r + rz*v_cl) / ((r^2) * cos(los_2));

pn_constant = 5;
ach = pn_constant * v_cl * los_1_rate;
acv = pn_constant * v_cl * los_2_rate;

% 89
drh_rx = rx / rh;
drh_ry = ry / rh;
dr_rx = rx / r;
dr_ry = ry / r;
dr_rz = rz / r;

dVcl_rx = - (rx_dot*r^2 - (rx*rx_dot + ry*ry_dot + rz*rz_dot)*rx) / (r^3);
dVcl_ry = - (ry_dot*r^2 - (rx*rx_dot + ry*ry_dot + rz*rz_dot)*ry) / (r^3);
dVcl_rz = - (rz_dot*r^2 - (rx*rx_dot + ry*ry_dot + rz*rz_dot)*rz) / (r^3);

dVcl_rxdot = - rx / r;
dVcl_rydot = - ry / r;
dVcl_rzdot = - rz / r;


dlos1_rx = -ry*(cos(los_1)^2) / rx^2;
dlos1_ry = (cos(los_1)^2) / rx;

% 107
dlos2_rx = -rz*rx / (cos(los_2)*r^3);
dlos2_ry = -rz*ry / (cos(los_2)*r^3);
dlos2_rz = (r^2 - rz^2) / (cos(los_2)*r^3);

dlos1rate_rx = (ry_dot*rh^2 - (ry_dot * rx - rx_dot * ry)*2 *rh *drh_rx) / rh^4;
dlos1rate_ry = (- rx_dot*rh^2 - (ry_dot * rx - rx_dot * ry)*2 *rh *drh_ry) / rh^4;
dlos1rate_rxdot = -ry / rh^2;
dlos1rate_rydot = rx / rh^2;

% 116
dlos2rate_rx = ((rz_dot * dr_rx + rz*dVcl_rx)*(r^2*cos(los_2)) ...
	- (rz_dot*r + rz*v_cl) * (2*r*cos(los_2)*dr_rx - r^2*sin(los_2)*dlos2_rx)) / (r^4 * (cos(los_2)^2));
dlos2rate_ry = ((rz_dot * dr_ry + rz*dVcl_ry)*(r^2*cos(los_2)) ...
	- (rz_dot*r + rz*v_cl) * (2*r*cos(los_2)*dr_ry - r^2*sin(los_2)*dlos2_ry)) / (r^4 * (cos(los_2)^2));
dlos2rate_rz = ((rz_dot * dr_rz + rz*dVcl_rz)*(r^2*cos(los_2)) ...
	- (rz_dot*r + rz*v_cl) * (2*r*cos(los_2)*dr_rz - r^2*sin(los_2)*dlos2_rz)) / (r^4 * (cos(los_2)^2));

% 122
dlos2rate_rxdot = rz*dVcl_rxdot / (r^2*cos(los_2));
dlos2rate_rydot = rz*dVcl_rydot / (r^2*cos(los_2));
dlos2rate_rzdot = (r + rz * dVcl_rzdot) / (r^2*cos(los_2));

%126
dach_rx = pn_constant * (dVcl_rx*los_1_rate + v_cl * dlos1rate_rx);
dach_ry = pn_constant * (dVcl_ry*los_1_rate + v_cl * dlos1rate_ry);
dach_rz = pn_constant * (dVcl_rz*los_1_rate);

%129
dach_rxdot = pn_constant * (dVcl_rxdot*los_1_rate + v_cl*dlos1rate_rxdot);
dach_rydot = pn_constant * (dVcl_rydot*los_1_rate + v_cl*dlos1rate_rydot);
dach_rzdot = pn_constant * (dVcl_rzdot*los_1_rate);

% 133
dacv_rx = pn_constant * (dVcl_rx*los_2_rate + v_cl * dlos2rate_rx);
dacv_ry = pn_constant * (dVcl_ry*los_2_rate + v_cl * dlos2rate_ry);
dacv_rz = pn_constant * (dVcl_rz*los_2_rate + v_cl * dlos2rate_rz);
dacv_rxdot = pn_constant * (dVcl_rxdot*los_2_rate + v_cl*dlos2rate_rxdot);
dacv_rydot = pn_constant * (dVcl_rydot*los_2_rate + v_cl*dlos2rate_rydot);
dacv_rzdot = pn_constant * (dVcl_rzdot*los_2_rate + v_cl*dlos2rate_rzdot);


% 140
drx2dot_rx = dach_rx * sin(los_1) + ach * cos(los_1) * dlos1_rx + ...
	dacv_rx * cos(los_1) * sin(los_2) + ...
	acv * (-sin(los_1) * sin(los_2) * dlos1_rx + cos(los_1) * cos(los_2) * dlos2_rx);

drx2dot_ry = dach_ry * sin(los_1) + ach * cos(los_1) * dlos1_ry + ...
	dacv_ry * cos(los_1) * sin(los_2) + ...
	acv * (-sin(los_1) * sin(los_2) * dlos1_ry + cos(los_1) * cos(los_2) * dlos2_ry);

drx2dot_rz = dach_rz * sin(los_1) + ...
	dacv_rz * cos(los_1) * sin(los_2) + acv * (cos(los_1)*cos(los_2)*dlos2_rz);

% 151
drx2dot_rxdot = dach_rxdot * sin(los_1) + dacv_rxdot * cos(los_1) * sin(los_2);
drx2dot_rydot = dach_rydot * sin(los_1) + dacv_rydot * cos(los_1) * sin(los_2);
drx2dot_rzdot = dach_rzdot * sin(los_1) + dacv_rzdot * cos(los_1) * sin(los_2);

% 155
dry2dot_rx = -dach_rx*cos(los_1) + ach*sin(los_1)*dlos1_rx + ...
	dacv_rx * sin(los_1) * sin(los_2) + ...
	acv*(cos(los_1)*sin(los_2) * dlos1_rx + sin(los_1) * cos(los_2) * dlos2_rx);

dry2dot_ry = -dach_ry*cos(los_1) + ach*sin(los_1)*dlos1_ry + ...
	dacv_ry * sin(los_1) * sin(los_2) + ...
	acv*(cos(los_1)*sin(los_2) * dlos1_ry + sin(los_1) * cos(los_2) * dlos2_ry);

dry2dot_rz = -dach_rz * cos(los_1) + ...
	dacv_rz * sin(los_1) * sin(los_2) + acv * (sin(los_1)*cos(los_2)*dlos2_rz);

% 166
dry2dot_rxdot = dach_rxdot * cos(los_1) + dacv_rxdot * sin(los_1) * sin(los_2);
dry2dot_rydot = dach_rydot * cos(los_1) + dacv_rydot * sin(los_1) * sin(los_2);
dry2dot_rzdot = dach_rzdot * cos(los_1) + dacv_rzdot * sin(los_1) * sin(los_2);

% 170
drz2dot_rx = -dacv_rx * cos(los_2) + acv*sin(los_2) * dlos2_rx;
drz2dot_ry = -dacv_ry * cos(los_2) + acv*sin(los_2) * dlos2_ry;
drz2dot_rz = -dacv_rz * cos(los_2) + acv*sin(los_2) * dlos2_rz;

drz2dot_rxdot = -dacv_rxdot*cos(los_2);
drz2dot_rydot = -dacv_rydot*cos(los_2);
drz2dot_rzdot = -dacv_rzdot*cos(los_2);

A_lin	= [zeros(3) eye(3);
	drx2dot_rx drx2dot_ry drx2dot_rz drx2dot_rxdot drx2dot_rydot drx2dot_rzdot;
	dry2dot_rx dry2dot_ry dry2dot_rz dry2dot_rxdot dry2dot_rydot dry2dot_rzdot;
	drz2dot_rx drz2dot_ry drz2dot_rz drz2dot_rxdot dry2dot_rydot drz2dot_rzdot];
%***** INSERT LINEARIZED EQUATIONS OF MOTION (RELATIVE MOTION BETWEEN
%TARGET AND MISSILE) FOR A_lin. THE RELATIVE MOTION WILL DEPEND ON THE
%TARGET MANEVUER, AS WELL THE MISSILE PN GUIDANCE LAW.

%----- Computations of Kalman gain and estimation error covariance update
P_seeker	= reshape( state_seeker_covar, 6, 6);
Kalman_gain	= P_seeker * C_skr' * parameters_system.noise.Rinv_skr;

state_dot_seeker_est= [state_seeker_est(4:6); ...
	ach*sin(los_1) + acv*cos(los_1)*sin(los_2); ...
	-ach*cos(los_1) + acv*sin(los_1)*sin(los_2); -acv*cos(los_2)] + ...
	Kalman_gain * (seeker_meas - C_skr*state_seeker_est);

state_dot_seeker_covar	= (A_lin - Kalman_gain*C_skr)*P_seeker + ...
	P_seeker*(A_lin - Kalman_gain*C_skr)' + ...
	Kalman_gain*parameters_system.noise.R_skr*Kalman_gain' + ...
	parameters_system.noise.Q_skr;
state_dot_seeker_covar	= state_dot_seeker_covar(:);