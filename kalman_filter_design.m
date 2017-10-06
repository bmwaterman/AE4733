clear variables
syms gyro_x gyro_y gyro_z mag_x mag_y mag_z psi theta phi psid thetad phid real

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

%% EKF
x = [psi theta phi psid thetad phid]';

f = x + [x(4:6); 0; 0; 0];
F = jacobian(f, x)

z = [gyro_x gyro_y gyro_z mag_x mag_y mag_z]';
h = [omega_ie_e; mag_e];
H = jacobian(h, x)



