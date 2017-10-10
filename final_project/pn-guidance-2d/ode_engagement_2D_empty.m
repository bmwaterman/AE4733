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

function xi_dot = ode_engagement_2D_empty(t, xi)

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


% Fill in PN law here.. obtain ft_missile_ct, ft_missile_cn



xi_dot_target	= ode_particle_2D(t, [pos_target; v_target; yaw_target], 0, 0);								% Target just moves along a straight line
xi_dot_missile	= ode_particle_2D(t, [pos_missile; v_missile; yaw_missile], ...
	ft_missile_ct, ft_missile_cn);											% Missile follows PN law

xi_dot		= [xi_dot_missile; xi_dot_target];