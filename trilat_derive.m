syms p_r_x p_r_y p_r_z p_i_x p_i_y p_i_z r_i epsilon_c real
p_r = [p_r_x p_r_y p_r_z]';
p_i = [p_i_x p_i_y p_i_z]';
f_i = norm(p_r-p_i) - r_i + epsilon_c;
gradient = jacobian(f_i, [p_r_x p_r_y p_r_z epsilon_c])