function euler321_angles_dot = rotational_kinematics(t, euler321_angles, omega_iee)

psi_yaw = euler321_angles(1);
theta_pitch = euler321_angles(2);
phi_roll = euler321_angles(3);

H_321 = [-sin(theta_pitch) 0 1; 
    sin(phi_roll)*cos(theta_pitch) cos(phi_roll) 0;
    cos(phi_roll)*cos(theta_pitch) -sin(phi_roll) 0];

euler321_angles_dot = H_321 \ omega_iee;
end



