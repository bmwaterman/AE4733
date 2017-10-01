%Create a function to write down the differnetial equations
function state_dot = some_name_of_function(t,state_xi, rate_gyro_meas, accel_meas, omega_icc);
%Time is always the first variable, state you're trying to integrate is
%second input

% need accel_meas_t from accel_meas
% need omega_iee from rate_gyros_meas
%pull out correct column

grav_tt = [0; 0; 9.81];
omega_icc = [0, 0, 2*pi/86400];

posn_tt = state_xi(1:3);
vel_tt = state_xi(4:6);
yaw_321 = state_xi(7);
pitch_321 = state_xi(8);
roll_321 = state_xi(9);

Rct = []; % 3-2 rotation involving latitude and longitude
Ret = []; % 321 rotation involving yaw, pitch, and roll
Rce = []; % define correctly from the previous two

omega_itt = Rct * omega_icc;

posn_dot = vel_tt;
vel_dot = Ret*accel_meas_t + grav_t - 2*cross(omega_itt, vel_tt);

angles_dot = H_321 \ (omega_iee - Rce*omega_icc);


