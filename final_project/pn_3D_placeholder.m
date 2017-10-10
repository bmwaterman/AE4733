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
3D proportional navigation guidance law (placeholder, currently this code
does nothing).
%}

function accel_body = pn_3D_placeholder(state_missile_est, state_seeker_est)


pos_rel		= state_seeker_est(1:3);
xdot_rel	= state_seeker_est(4);
ydot_rel	= state_seeker_est(5);
zdot_rel	= state_seeker_est(6);

yaw_missile	= state_missile_est(5);
ptc_missile	= state_missile_est(6);

%**** YOUR CODE FOR COMPUTING THE PN GUIDANCE LAW GOES HERE
%**** CAN COPY-PASTE FROM HW3 CODE

accel_body = [0; 0; 0]; %**** REMOVE THIS LINE AFTER YOU FINISH CODING THE PN GUIDANCE LAW