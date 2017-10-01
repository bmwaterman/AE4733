function euler321_angles_dot = midterm_kinematics(t, euler321_angles, omega_iee)

psi = euler321_angles(1);
theta = euler321_angles(2);
phi = euler321_angles(3);

H_321 = [cos(phi)*cos(theta) sin(phi) 0; 
    -sin(phi)*cos(theta) cos(phi) 0;
    sin(theta) 0 1];

euler321_angles_dot = H_321\omega_iee;
end

