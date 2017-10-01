%% homework 3
% 
clear variables, clc
%% Problem 4
p4_data = load('hw3_p4.mat');
y0 = [p4_data.ptt_0; p4_data.vtt_0; p4_data.yaw_0; p4_data.ptc_0; p4_data.rol_0];
[t_sim, y_sim] = ode45(@(t,y) ins_fun(t,y,p4_data), p4_data.time_stamp, y0);

figure();
plot3(y_sim(:,1),y_sim(:,2),y_sim(:,3));
title('position trajectory')

figure();
plot(t_sim, y_sim(:,4:6));
xlabel('time (sec)');
ylabel ('velocity (m/s)');
title('velocity');

figure();
plot(t_sim, rad2deg(y_sim(:,6:9)));
xlabel('time (sec)');
ylabel ('euler angles (deg)');
title('euler angles');
legend ('yaw(psi)','pitch(theta)','roll(phi)')


