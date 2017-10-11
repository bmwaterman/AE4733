%{
Copyright (c) 2014 Raghvendra V. Cowlagi. All rights reserved.

Copyright notice: 
=================
No part of this work may be reproduced without the written permission of
the copyright holder, except for non-profit and educational purposes under
the provisions of Title 17, USC Section 107 of the United States Copyright
Act of 1976. Reproduction of this work for commercial use is a violation of
copyright.

Author information:
===================
Raghvendra V. Cowlagi, Ph.D,
Assistant Professor, Aerospace Engineering Program,
Department of Mechanical Engineering, Worcester Polytechnic Institute.
 
Higgins Laboratories, 247,
100 Institute Road, Worcester, MA 01609.
Phone: +1-508-831-6405
Email: rvcowlagi@wpi.edu
Website: http://www.wpi.edu/~rvcowlagi

Program description:
====================
ODE pure pursuit and PN guidance schemes.
%}

function [value, isterminal, direction] = event_missilestrike_2D(t, xi)

pos_missile = xi(1:2);
v_missile	= xi(3);
yaw_missile	= xi(4);

pos_target	= xi(5:6);
v_target	= xi(7);
yaw_target	= xi(8);

xdot_missile= v_missile*cos(yaw_missile);
ydot_missile= v_missile*sin(yaw_missile);
xdot_target = v_target*cos(yaw_target);
ydot_target = v_target*sin(yaw_target);

r_miss		= norm(pos_target - pos_missile);

if r_miss < 1
	value(1,1)	= 0;
else
	value(1,1)	= r_miss;
end

v_closing = -( (pos_target(1) - pos_missile(1))*(xdot_target - xdot_missile) + ...
	(pos_target(2) - pos_missile(2))*(ydot_target - ydot_missile) ) / r_miss;
value(2, 1)		= v_closing;


isterminal(1,1)	= 1;
isterminal(2,1) = 1;
direction(1,1)	= 0;
direction(2,1)	= 0;
