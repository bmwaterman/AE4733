function [ v ] = v( ax(t), t)
ode45(@(t, y)v_dot(t, y, ax(t)), [0 60], [75; 0; 0;])


end

