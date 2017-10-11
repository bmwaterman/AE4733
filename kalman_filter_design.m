clear variables
syms gyro_x gyro_y gyro_z mag_x mag_y mag_z acc_x acc_y acc_z psi theta phi psid thetad phid dt real

%% rotations
% x rotation phi
R_b_e = [1 0 0;
         0 cos(phi) sin(phi);
         0 -sin(phi) cos(phi)];
     
% y rotation theta
R_a_b = [cos(theta) 0 -sin(theta);
         0 1 0;
         sin(theta) 0 cos(theta)];
         
% z rotation psi
R_i_a = [cos(psi) sin(psi) 0;
         -sin(psi) cos(psi) 0;
         0 0 1];

%% get yaw from mag
% mag_e = [mag_x mag_y mag_z]';
% mag_a = R_a_b' * R_b_e' * mag_e;
% psi_mag = atan2(mag_a(2), mag_a(1))

mag_i = [1 0 0]';
mag_e = R_b_e * R_a_b * R_i_a * mag_i;

%% get psid thetad phid from gyro
% omega_ie_i = rotational_kinematics(0, [psi theta phi]', [gyro_z gyro_y gyro_x]')
% psid_gyro = omega_ie_i(1);
% thetad_gyro = omega_ie_i(2);
% phid_gyro = omega_ie_i(3);
omega_ie_e = R_b_e * R_a_b * [0 0 psid]' + R_b_e * [0 thetad 0]' + [phid 0 0]'

%% use gravity to keep track of the orientation...
acc_e = R_b_e * R_a_b * R_i_a * [0 0 1]';

%% EKF
x = [psi theta phi psid thetad phid]';

f = x + dt * [x(4:6); 0; 0; 0];
F = jacobian(f, x)

% z = [gyro_x gyro_y gyro_z mag_x mag_y mag_z]';
h = [omega_ie_e; mag_e; acc_e];
H = jacobian(h, x)

%%
close all

n_states = 6;
n_measurements = 9;
x_prev = zeros(6,1);
u_prev = 0.5;
P_prev = 0;
Q = diag(ones(n_states,1)) * 0.01;
R = diag(ones(n_measurements,1)) * 0.01;

data = csvread('hw3q7.csv',1,0);
time_stamp = data(:, 1)';
acc_readings = data(:, 2:4)';
gyro_readings = data(:, 5:7)'; % csv data is roll pitch yaw
mag_readings = data(:, 8:10)';

%hack
mag_readings(3, :) = zeros(1, size(mag_readings,2));

normalize = @(v) v/norm(v);

for col = 1:size(mag_readings,2)
    mag_readings(:, col) = normalize(mag_readings(:, col));
end

for col = 1:size(acc_readings,2)
    acc_readings(:, col) = normalize(acc_readings(:, col));
end

z = [gyro_readings; mag_readings; acc_readings];
f_fun = matlabFunction(f, 'Vars', {x,dt});
F_fun = matlabFunction(F, 'Vars', {x,dt});
h_fun = matlabFunction(h, 'Vars', {x});
H_fun = matlabFunction(H, 'Vars', {x});

x_traj = zeros(6, size(z,2));

for t = 1:size(z,2)
    [ x_estimate, P_estimate ] = kalman_sensor_fusion(x_prev, u_prev, P_prev, f_fun, F_fun, h_fun, H_fun, z(:, t), Q, R);
    x_traj(:, t) = x_estimate;
    x_prev = x_estimate;
    P_prev = P_estimate;
end

plot(time_stamp, x_traj(1:3,:))
figure();
plot(time_stamp, x_traj(4:6,:))


