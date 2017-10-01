function [ v ] = midterm_v( t)
v = ode45(@(t, y)v_dot(t, y, ax), [0 60], [75; 0; 0;])


end

