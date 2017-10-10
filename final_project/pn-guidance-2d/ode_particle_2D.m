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

Program description: Particle dynamics equations of motion.
%}

function xi_dot = ode_particle_2D(t, xi, ft, fn)


v		= xi(3);
thta	= xi(4);

xi_dot	= zeros(4, 1);

xi_dot(1)	= v*cos(thta);
xi_dot(2)	= v*sin(thta);
xi_dot(3)	= ft;
xi_dot(4)	= fn / v;

