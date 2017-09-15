clear all, close all, clc
psi = psi 
theta = theta 
phi = phi 

v_e = [1, 0, 0]'

R_i_to_a = [1, 0, 0; 0, cos(psi), sin(psi); 0, -sin(psi), cos(psi)]
R_a_to_b = [cos(theta), sin(theta), 0; -sin(theta), cos(theta), 0; 0, 0, 1]
R_b_to_e = [cos(phi), 0, -sin(phi); 0, 1, 0; sin(phi), 0, cos(phi)]

R_i_to_e = R_b_to_e * R_a_to_b * R_i_to_a

R_e_to_i = R_i_to_e'

v_i = R_e_to_i * v_e

