%Create a function to write down the differnetial equations
function state_dot = ins_fun(t,state_xi, DATA)
%Time is always the first variable, state you're trying to integrate is
%second input

% need accel_meas_t from accel_meas
% need omega_iee from rate_gyros_meas
%pull out correct column
current_timestamp_col = find((t >= DATA.time_stamp), 1, 'last');
accel_meas_t = DATA.accel_readings(:, current_timestamp_col);
omega_iee = DATA.gyro_readings(:, current_timestamp_col);

grav_tt = [0; 0; 9.81];
omega_icc = [0, 0, 2*pi/86400]';

% posn_tt = state_xi(1:3);
vel_tt = state_xi(4:6);
yaw_321 = state_xi(7);
pitch_321 = state_xi(8);
roll_321 = state_xi(9);

mu_lat = deg2rad(42.271167);
mu_long = deg2rad(-71.807627);

% 3-2 rotation involving latitude and longitude
Rct = [cos(-(pi/2 + mu_lat)) 0 -sin(-(pi/2 + mu_lat));
        0 1 0;
        sin(-(pi/2 + mu_lat)) 0 cos(-(pi/2 + mu_lat))] * ...
      [cos(mu_long) sin(mu_long) 0;
      -sin(mu_long) cos(mu_long) 0;
      0 0 1];

Rte = ...
    [1 0 0;
    0 cos(roll_321) sin(roll_321);
    0 -sin(roll_321) cos(roll_321)] * ...
    [cos(pitch_321) 0 -sin(pitch_321);
    0 1 0;
    sin(pitch_321) 0 cos(pitch_321)] * ...
    [cos(yaw_321) sin(yaw_321) 0;
    -sin(yaw_321) cos(yaw_321) 0;
    0 0 1];

% Ret = inv(Rte); % 321 rotation involving yaw, pitch, and roll
Rce = Rte * Rct; % define correctly from the previous two

omega_itt = Rct * omega_icc;

H_321 = [-sin(pitch_321) 0 1;
    sin(roll_321)*cos(pitch_321) cos(roll_321) 0;
    cos(roll_321)*cos(pitch_321) -sin(roll_321) 0];

posn_dot = vel_tt;
vel_dot = Rte \ accel_meas_t + grav_tt - 2*cross(omega_itt, vel_tt);
angles_dot = H_321 \ (omega_iee - Rce*omega_icc);

state_dot = [posn_dot; vel_dot; angles_dot];

