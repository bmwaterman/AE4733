function [ acceleration_cmd_body ] = pn_guidance_law_3d( p_and_p_dot, yaw, ptc )

Rte = @(yaw_321, pitch_321, roll_321) ...
    [1 0 0;
    0 cos(roll_321) sin(roll_321);
    0 -sin(roll_321) cos(roll_321)] * ...
    [cos(pitch_321) 0 -sin(pitch_321);
    0 1 0;
    sin(pitch_321) 0 cos(pitch_321)] * ...
    [cos(yaw_321) sin(yaw_321) 0;
    -sin(yaw_321) cos(yaw_321) 0;
    0 0 1];

N1 = 5;
N2 = 5;

alpha_dot_fun = @(in1)(in1(1,:).*in1(5,:)-in1(4,:).*in1(2,:))./(in1(1,:).^2+in1(2,:).^2);
beta_dot_fun =  @(in1)1.0./sqrt((in1(1,:).^2+in1(2,:).^2)./(in1(1,:).^2+in1(2,:).^2+in1(3,:).^2)).*1.0./(in1(1,:).^2+in1(2,:).^2+in1(3,:).^2).^(3.0./2.0).*(in1(1,:).^2.*in1(6,:)+in1(2,:).^2.*in1(6,:)-in1(4,:).*in1(1,:).*in1(3,:)-in1(5,:).*in1(2,:).*in1(3,:));
v_c_fun = @(in1)-(in1(4,:).*in1(1,:)+in1(5,:).*in1(2,:)+in1(6,:).*in1(3,:)).*1.0./sqrt(in1(1,:).^2+in1(2,:).^2+in1(3,:).^2);

alpha_dot = alpha_dot_fun(p_and_p_dot);
beta_dot = beta_dot_fun(p_and_p_dot);
v_c = v_c_fun(p_and_p_dot);

a_ch = N1 * v_c * alpha_dot;
a_cv = N2 * v_c * beta_dot;

% fix me
acceleration_cmd_body = [a_ch a_cv];


a_ch_i = [0 0 1]' * a_ch;
a_cv_i = Rte(alpha beta 0) * [0 0 1]' * a_cv;

a_c_i = a_ch_i + a_cv_i;

a_c_e = 








end

