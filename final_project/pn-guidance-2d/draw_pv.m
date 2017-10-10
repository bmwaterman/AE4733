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
Plot position and velocity in 2D and 3D.
%}

function ae4733_draw_pv(pos, vel, mode, color, qs)

v	= vel(1);
yaw	= vel(2);
if strcmp(mode, '3D')
	ptc = vel(3);
end

if strcmp(mode, '2D')
	plot(pos(1), pos(2), 'Marker', 'o', 'MarkerEdgeColor', color, ...
		'MarkerFaceColor', color, 'MarkerSize', 5);
	quiver(pos(1), pos(2), qs*v*cos(yaw), qs*v*sin(yaw), ...
		'LineWidth', 2, 'Color', color, 'MaxHeadSize', 10);
else
	plot3(pos(1), pos(2), pos(3), 'Marker', 'o', 'MarkerEdgeColor', color, ...
		'MarkerFaceColor', color, 'MarkerSize', 5);
	quiver3(pos(1), pos(2), pos(3), ...
		qs*v*cos(yaw)*cos(ptc), qs*v*sin(yaw)*cos(ptc), -qs*v*sin(ptc), ...
		'LineWidth', 2, 'Color', color, 'MaxHeadSize', 10);
end