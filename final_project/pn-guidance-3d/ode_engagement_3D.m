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
ODE for 3D proportional navigation simulation.
%}

function xi_dot = ode_engagement_3D(t, xi,target_maneuver_num,target_accel_lat,target_accel_vrt)

%----- Readability
pos_missile = xi(1:3);
v_missile	= xi(4);
yaw_missile	= xi(5);
ptc_missile	= xi(6);

pos_target	= xi(7:9);
v_target	= xi(10);
yaw_target	= xi(11);
ptc_target	= xi(12);

xdot_rel	= v_target*cos(yaw_target)*cos(ptc_target) - ...
	v_missile*cos(yaw_missile)*cos(ptc_missile);
ydot_rel	= v_target*sin(yaw_target)*cos(ptc_target) - ...
	v_missile*sin(yaw_missile)*cos(ptc_missile);
zdot_rel	= -v_target*sin(ptc_target) + v_missile*sin(ptc_missile);

pos_rel		= pos_target - pos_missile;



%**** INSERT THE NAME OF GUIDANCE LAW FUNCTION HERE*****
fh_pn_guidance_law = @pn_guidance_law_3d; % Replace the '[]' with: '@name_of_your_function' (w/o quotation marks)
%{
Your guidance law must be written as a MATLAB function that has:
1. Inputs = [Relative position of target (3 intertial coord); 
			 Relative velocity of target (3 inertial coord)];
	in that order
2. Outputs= Commanded inertial acceleration expressed in coordinates
	attached to the missile body axes. 
%}

% missile_accel_body_cmd	= [0;0;0];

%**** UNCOMMENT THE FOLLOWING LINE WHEN TESTING YOUR GUIDANCE LAW ****
missile_accel_body_cmd	= Rte(yaw_missile, ptc_missile, 0) \ fh_pn_guidance_law([pos_rel; xdot_rel; ydot_rel; zdot_rel]);

switch target_maneuver_num
	case 1
		%----- Straight and level: no accelerations, zero angle of climb
		target_accel_body	= [0; 0; 0];
	case 2
		%----- Steady climb/descent: no accelerations, non-zero angle of
		%		climb/descent 
		target_accel_body	= [0; 0; 0];
	case 3
		%----- Level turn: non-zero lateral acceleration, no tangential or
		%		vertical acceleration, zero angle of climb
		target_accel_body	= [0; target_accel_lat; 0];
	case 4
		%----- Climbing/descending turn: non-zero lateral acceleration, no
		%		tangential or vertical acceleration, non-zero angle of climb
		target_accel_body	= [0; target_accel_lat; 0];
	case 5
		%----- Pull-up/pull-down: non-zero vertical acceleration , no
		%		tangential or lateral acceleration
		target_accel_body	= [0; 0; target_accel_vrt];
end

xi_dot_target	= ode_particle_3D(t, [pos_target; v_target; yaw_target; ptc_target], target_accel_body);
xi_dot_missile	= ode_particle_3D(t, [pos_missile; v_missile; yaw_missile; ptc_missile], missile_accel_body_cmd);

xi_dot		= [xi_dot_missile; xi_dot_target];