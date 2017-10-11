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


%%
H_321 = @(theta, phi) [-sin(theta) 0 1; 
    sin(phi)*cos(theta) cos(phi) 0;
    cos(phi)*cos(theta) -sin(phi) 0];

x = [psi theta phi]';
u = [gyro_x gyro_y gyro_z dt]';
f_expr = x + (H_321(theta, phi) \ u(1:3, :)) * u(4, :);
h_expr = eye(3) * x;
F_expr = jacobian(f_expr, x);
H_expr = jacobian(h_expr, x);

%%
close all

n_states = 3;
n_measurements = 3;
x_prev = [0 0 pi]';
u_prev = [0 0 0 0.02]';
P_prev = eye(n_states);
Q = eye(n_states) * 1e-2;
R = eye(n_measurements,1) * 1e-0;

data = csvread('hw3q7_2.csv',1,0);
time_stamp = data(:, 1)';
% acc_readings = data(:, 2:4)';
gyro_readings = data(:, 5:7)';
% mag_readings = data(:, 8:10)';
heading = deg2rad(data(:, 13:-1:11)');
dt = [0.02, diff(time_stamp)];

z_traj = heading;
u_traj = [gyro_readings; dt];
f_fun = matlabFunction(f_expr, 'Vars', {x,u});
F_fun = matlabFunction(F_expr, 'Vars', {x,u});
h_fun = matlabFunction(h_expr, 'Vars', {x});
H_fun = matlabFunction(H_expr, 'Vars', {x});

x_traj = zeros(n_states, size(z_traj,2));

for t = 2:size(z_traj,2)
    [ x_estimate, P_estimate ] = kalman_sensor_fusion(x_prev, u_traj(:, t-1), P_prev, f_fun, F_fun, h_fun, H_fun, z_traj(:, t), Q, R);
    x_traj(:, t) = x_estimate;
    x_prev = x_estimate;
    P_prev = P_estimate;
end

plot(time_stamp, x_traj(1:3,:))


