clear all, close all, clc
psi = sym('psi') 
theta = sym('theta') 
phi = sym('phi') 

v_e = [1, 0, 0]'

R_i_to_a = [1, 0, 0; 0, cos(psi), sin(psi); 0, -sin(psi), cos(psi)]
R_a_to_b = [cos(theta), 0, -sin(theta); 0, 1, 0; sin(theta), 0, cos(theta)]
R_b_to_e = [cos(phi), sin(phi), 0; -sin(phi), cos(phi), 0; 0, 0, 1]

R_i_to_e = R_b_to_e * R_a_to_b * R_i_to_a;

R_e_to_i = R_i_to_e';

v_i = R_e_to_i * v_e;

psi_dot = sym('psi_dot')
theta_dot = sym('theta_dot')
phi_dot = sym('phi_dot')

omega_ia_a = [psi_dot; 0; 0;]
omega_ab_b = [0; theta_dot; 0;]
omega_be_e = [0; 0; phi_dot;]

omega_iee = ((R_b_to_e * R_a_to_b) * omega_ia_a) + (R_b_to_e * omega_ab_b) + omega_be_e

omega_iee = [cos(phi)*cos(theta) sin(phi) 0; -cos(theta)*sin(phi) cos(phi) 0; sin(theta) 0 1;]











