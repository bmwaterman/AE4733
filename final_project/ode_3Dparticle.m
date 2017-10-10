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

function xi_dot = ode_3Dparticle(t, xi, u, parameters_particle)

% xi(1:3) = inertial (x,y,z) coordinates of inertial position
airspeed_V		= xi(4);
angle_heading	= xi(5);
angle_climb		= xi(6);

xi_dot	= zeros(6, 1);

%----- Saturation
if nargin > 3
	max_accel_long	= parameters_particle.max_accel_long;
	max_accel_lat	= parameters_particle.max_accel_lat;
else
	max_accel_long	= Inf;
	max_accel_lat	= Inf;
end
accel_body(1, 1)	= min( max(u(1), -max_accel_long), max_accel_long);
accel_lat			= min( max(norm(u(2:3)), -max_accel_lat),  ...
	max_accel_lat);
if norm(u(2:3)) > max_accel_lat
	accel_body(2:3, 1)	= u(2:3) * (accel_lat/norm(u(2:3)));
else
	accel_body(2:3, 1)	= u(2:3);
end

%----- Equations of motion
xi_dot(1)	= airspeed_V*cos(angle_heading)*cos(angle_climb);
xi_dot(2)	= airspeed_V*sin(angle_heading)*cos(angle_climb);
xi_dot(3)	= -airspeed_V*sin(angle_climb);
xi_dot(4)	= accel_body(1);
xi_dot(5)	= accel_body(2) / (airspeed_V*cos(angle_climb));
xi_dot(6)	= -accel_body(3) / airspeed_V;