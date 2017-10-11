%{
Copyright (c) 2014 Raghvendra V. Cowlagi. All rights reserved.

Copyright notice: 
=================
No part of this work may be reproduced without the written permission of
the copyright holder, except for non-profit and educational purposes under
the provisions of Title 17, USC Section 107 of the United States Copyright
Act of 1976. Reproduction of this work for commercial use is a violation of
copyright.


Disclaimer:
===========
This software program is intended for educational and research purposes.
The author and the institution with which the author is affiliated are not
liable for damages resulting the application of this program, or any
section thereof, which may be attributed to any errors that may exist in
this program.


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


The author welcomes questions, comments, suggestions for improvements, and
reports of errors in this program.


Program description:
====================
Equations of motion for 3D particle with body x-axis in the direction of
the velocity vector.
%}

function xi_dot = ode_particle_3D(t, xi, accel_body)

V	= xi(4);
yaw	= xi(5);
ptc	= xi(6);

xi_dot	= zeros(6, 1);

xi_dot(1)	= V*cos(yaw)*cos(ptc);
xi_dot(2)	= V*sin(yaw)*cos(ptc);
xi_dot(3)	= -V*sin(ptc);
xi_dot(4)	= accel_body(1);
xi_dot(5)	= accel_body(2) / (V*cos(ptc));
xi_dot(6)	= -accel_body(3) / V;

