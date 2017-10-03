function [ psii ] = midterm_psii( t) 
psii = ode45(@(t, y)psii_dot(t, y, ay), [0 60], [0 0 45*pi/180])


end

