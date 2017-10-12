function [ acceleration_cmd_inertial ] = pn_guidance_law_3d( p_and_p_dot )

N1 = 50;
N2 = N1;

alpha_dot_fun = @(in1)(in1(1,:).*in1(5,:)-in1(4,:).*in1(2,:))./(in1(1,:).^2+in1(2,:).^2);
beta_dot_fun =  @(in1)1.0./sqrt((in1(1,:).^2+in1(2,:).^2)./(in1(1,:).^2+in1(2,:).^2+in1(3,:).^2)).*1.0./(in1(1,:).^2+in1(2,:).^2+in1(3,:).^2).^(3.0./2.0).*(in1(1,:).^2.*in1(6,:)+in1(2,:).^2.*in1(6,:)-in1(4,:).*in1(1,:).*in1(3,:)-in1(5,:).*in1(2,:).*in1(3,:));
v_c_fun = @(in1)-(in1(4,:).*in1(1,:)+in1(5,:).*in1(2,:)+in1(6,:).*in1(3,:)).*1.0./sqrt(in1(1,:).^2+in1(2,:).^2+in1(3,:).^2);

alpha = atan2(p_and_p_dot(2), p_and_p_dot(1));
% alpha = atan(p_and_p_dot(2) / p_and_p_dot(1));
beta = asin(p_and_p_dot(3) / norm(p_and_p_dot(1:3)));

alpha_dot = alpha_dot_fun(p_and_p_dot);
beta_dot = beta_dot_fun(p_and_p_dot);
v_c = v_c_fun(p_and_p_dot);

a_ch = N1 * v_c * alpha_dot;
a_cv = N2 * v_c * beta_dot;


a_ch_i = Rte(alpha, beta, 0) \ ([0 1 0]' * a_ch);
a_cv_i = Rte(alpha, beta, 0) \ ([0 0 1]' * a_cv);

acceleration_cmd_inertial = a_ch_i + a_cv_i;

end

