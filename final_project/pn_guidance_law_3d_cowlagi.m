function [ acceleration_cmd_inertial ] = pn_guidance_law_3d_cowlagi( state_seeker_est )
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
a_ch = pn_constant * v_cl * los_1_rate;
a_cv = pn_constant * v_cl * los_2_rate;

a_ch_i = Rte(los_1, los_2, 0) \ ([0 1 0]' * a_ch);
a_cv_i = Rte(los_1, los_2, 0) \ ([0 0 1]' * a_cv);

acceleration_cmd_inertial = a_ch_i + a_cv_i;
end