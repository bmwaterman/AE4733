function A = A_fun(in1)
%A_FUN
%    A = A_FUN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    11-Oct-2017 18:55:03

x = in1(1,:);
xd = in1(4,:);
y = in1(2,:);
yd = in1(5,:);
z = in1(3,:);
zd = in1(6,:);
t2 = x.^2;
t3 = y.^2;
t4 = t2+t3;
t5 = x.*yd;
t14 = xd.*y;
t6 = t5-t14;
t7 = 1.0./t4.^2;
t8 = 1.0./x.^2;
t9 = t4.*t8;
t10 = sqrt(t9);
t11 = z.^2;
t12 = t2+t3+t11;
t13 = 1.0./sqrt(t12);
t15 = x.*xd;
t16 = y.*yd;
t17 = z.*zd;
t18 = t15+t16+t17;
t19 = abs(x);
t20 = abs(y);
t21 = abs(z);
t22 = 1.0./t12;
t23 = t4.*t22;
t24 = 1.0./sqrt(t23);
t25 = t19.^2;
t26 = t20.^2;
t27 = t21.^2;
t28 = t25+t26+t27;
t29 = 1.0./sqrt(t28);
t30 = t2.*zd;
t31 = t3.*zd;
t35 = x.*xd.*z;
t36 = y.*yd.*z;
t32 = t30+t31-t35-t36;
t33 = 1.0./t4;
t34 = 1.0./t12.^2;
t37 = 1.0./sqrt(t9);
t38 = 1.0./x.^3;
t39 = t4.*t38.*2.0;
t40 = 1.0./x;
t47 = t40.*2.0;
t41 = t39-t47;
t42 = 1.0./t4.^3;
t43 = 1.0./t12.^(3.0./2.0);
t44 = 1.0./t12.^3;
t45 = 1.0./t23.^(3.0./2.0);
t46 = 1.0./t28.^(3.0./2.0);
t48 = x.*zd.*2.0;
t64 = xd.*z;
t49 = t48-t64;
t50 = t22.*x.*2.0;
t69 = t4.*t34.*x.*2.0;
t51 = t50-t69;
t52 = sign(x);
t53 = t2.*t6.*t10.*t13.*t18.*t42.*y.*2.0e1;
t54 = t2.*t6.*t7.*t10.*t18.*t43.*y.*5.0;
t55 = y.*zd.*2.0;
t70 = yd.*z;
t56 = t55-t70;
t57 = t22.*y.*2.0;
t71 = t4.*t34.*y.*2.0;
t58 = t57-t71;
t59 = sign(y);
t60 = 1.0./t12.^4;
t61 = t15+t16;
t62 = sign(z);
t63 = t2.*t10.*t24.*t29.*t32.*t33.*t34.*y.*z.*5.0;
t65 = 1.0./t28;
t66 = -t11+t25+t26+t27;
t67 = t65.*t66;
t68 = sqrt(t67);
t72 = 1.0./sqrt(t67);
t73 = 1.0./t28.^2;
A = reshape([0.0,0.0,0.0,t53+t54-t6.*t7.*t10.*t13.*t18.*y.*5.0-t6.*t7.*t10.*t13.*x.*xd.*y.*5.0-t7.*t10.*t13.*t18.*x.*y.*yd.*5.0+t6.*t7.*t13.*t18.*t37.*t41.*x.*y.*(5.0./2.0)+t2.*t10.*t18.*t24.*t29.*t33.*t34.*t49.*z.*5.0+t10.*t18.*t24.*t29.*t32.*t33.*t34.*x.*z.*1.0e1+t2.*t10.*t24.*t29.*t32.*t33.*t34.*xd.*z.*5.0-t2.*t18.*t24.*t29.*t32.*t33.*t34.*t37.*t41.*z.*(5.0./2.0)-t2.*t10.*t18.*t29.*t32.*t33.*t34.*t45.*t51.*z.*(5.0./2.0)-t2.*t7.*t10.*t18.*t24.*t29.*t32.*t34.*x.*z.*1.0e1-t2.*t10.*t18.*t24.*t29.*t32.*t33.*t44.*x.*z.*2.0e1-t2.*t10.*t18.*t19.*t24.*t32.*t33.*t34.*t46.*t52.*z.*5.0,t6.*t7.*t10.*t13.*t18.*x.*1.0e1+t2.*t6.*t7.*t10.*t13.*xd.*5.0+t2.*t7.*t10.*t13.*t18.*yd.*5.0-t2.*t6.*t7.*t13.*t18.*t37.*t41.*(5.0./2.0)-t2.*t6.*t7.*t10.*t18.*t43.*x.*5.0-t2.*t6.*t10.*t13.*t18.*t42.*x.*2.0e1+t10.*t18.*t24.*t29.*t32.*t33.*t34.*y.*z.*5.0-t2.*t7.*t10.*t18.*t24.*t29.*t32.*t34.*y.*z.*1.0e1-t2.*t10.*t18.*t24.*t29.*t32.*t33.*t44.*y.*z.*2.0e1+t10.*t18.*t24.*t29.*t33.*t34.*t49.*x.*y.*z.*5.0+t10.*t24.*t29.*t32.*t33.*t34.*x.*xd.*y.*z.*5.0-t18.*t24.*t29.*t32.*t33.*t34.*t37.*t41.*x.*y.*z.*(5.0./2.0)-t10.*t18.*t29.*t32.*t33.*t34.*t45.*t51.*x.*y.*z.*(5.0./2.0)-t10.*t18.*t19.*t24.*t32.*t33.*t34.*t46.*t52.*x.*y.*z.*5.0,t18.*t24.*t34.*t49.*t68.*5.0+t24.*t32.*t34.*t68.*xd.*5.0-t18.*t32.*t34.*t45.*t51.*t68.*(5.0./2.0)-t18.*t24.*t32.*t44.*t68.*x.*2.0e1+t18.*t24.*t32.*t34.*t72.*(t19.*t52.*t65.*2.0-t19.*t52.*t66.*t73.*2.0).*(5.0./2.0),0.0,0.0,0.0,t6.*t7.*t10.*t13.*t18.*x.*-5.0-t3.*t6.*t7.*t13.*t18.*t37.*t40.*5.0+t3.*t6.*t7.*t10.*t18.*t43.*x.*5.0+t3.*t6.*t10.*t13.*t18.*t42.*x.*2.0e1+t7.*t10.*t13.*t18.*x.*xd.*y.*5.0-t6.*t7.*t10.*t13.*x.*y.*yd.*5.0+t2.*t10.*t18.*t24.*t29.*t33.*t34.*t56.*z.*5.0+t18.*t24.*t29.*t32.*t33.*t34.*t37.*y.*z.*5.0+t2.*t10.*t24.*t29.*t32.*t33.*t34.*yd.*z.*5.0-t2.*t10.*t18.*t29.*t32.*t33.*t34.*t45.*t58.*z.*(5.0./2.0)-t2.*t7.*t10.*t18.*t24.*t29.*t32.*t34.*y.*z.*1.0e1-t2.*t10.*t18.*t24.*t29.*t32.*t33.*t44.*y.*z.*2.0e1-t2.*t10.*t18.*t20.*t24.*t32.*t33.*t34.*t46.*t59.*z.*5.0,-t53-t54-t2.*t7.*t10.*t13.*t18.*xd.*5.0+t6.*t7.*t13.*t18.*t37.*y.*5.0+t2.*t6.*t7.*t10.*t13.*yd.*5.0+t10.*t18.*t24.*t29.*t32.*t33.*t34.*x.*z.*5.0+t3.*t18.*t24.*t29.*t32.*t33.*t34.*t37.*t40.*z.*5.0-t3.*t7.*t10.*t18.*t24.*t29.*t32.*t34.*x.*z.*1.0e1-t3.*t10.*t18.*t24.*t29.*t32.*t33.*t44.*x.*z.*2.0e1+t10.*t18.*t24.*t29.*t33.*t34.*t56.*x.*y.*z.*5.0+t10.*t24.*t29.*t32.*t33.*t34.*x.*y.*yd.*z.*5.0-t10.*t18.*t29.*t32.*t33.*t34.*t45.*t58.*x.*y.*z.*(5.0./2.0)-t10.*t18.*t20.*t24.*t32.*t33.*t34.*t46.*t59.*x.*y.*z.*5.0,t18.*t24.*t34.*t56.*t68.*5.0+t24.*t32.*t34.*t68.*yd.*5.0-t18.*t32.*t34.*t45.*t58.*t68.*(5.0./2.0)-t18.*t24.*t32.*t44.*t68.*y.*2.0e1+t18.*t24.*t32.*t34.*t72.*(t20.*t59.*t65.*2.0-t20.*t59.*t66.*t73.*2.0).*(5.0./2.0),0.0,0.0,0.0,t6.*t7.*t10.*t13.*x.*y.*zd.*-5.0+t6.*t7.*t10.*t18.*t43.*x.*y.*z.*5.0+t2.*t10.*t18.*t24.*t29.*t32.*t33.*t34.*5.0+t2.*t10.*t11.*t18.*t29.*t32.*t45.*t60.*5.0-t2.*t10.*t11.*t18.*t24.*t29.*t32.*t33.*t44.*2.0e1-t2.*t10.*t18.*t24.*t29.*t33.*t34.*t61.*z.*5.0+t2.*t10.*t24.*t29.*t32.*t33.*t34.*z.*zd.*5.0-t2.*t10.*t18.*t21.*t24.*t32.*t33.*t34.*t46.*t62.*z.*5.0,t2.*t6.*t7.*t10.*t13.*zd.*5.0-t2.*t6.*t7.*t10.*t18.*t43.*z.*5.0+t10.*t18.*t24.*t29.*t32.*t33.*t34.*x.*y.*5.0+t10.*t11.*t18.*t29.*t32.*t45.*t60.*x.*y.*5.0-t10.*t11.*t18.*t24.*t29.*t32.*t33.*t44.*x.*y.*2.0e1-t10.*t18.*t24.*t29.*t33.*t34.*t61.*x.*y.*z.*5.0+t10.*t24.*t29.*t32.*t33.*t34.*x.*y.*z.*zd.*5.0-t10.*t18.*t21.*t24.*t32.*t33.*t34.*t46.*t62.*x.*y.*z.*5.0,t18.*t24.*t34.*t61.*t68.*-5.0+t24.*t32.*t34.*t68.*zd.*5.0-t18.*t24.*t32.*t44.*t68.*z.*2.0e1-t18.*t24.*t32.*t34.*t72.*(t65.*(z.*2.0-t21.*t62.*2.0)+t21.*t62.*t66.*t73.*2.0).*(5.0./2.0)+t4.*t18.*t32.*t45.*t60.*t68.*z.*5.0,1.0,0.0,0.0,t3.*t7.*t10.*t13.*t18.*x.*5.0-t2.*t6.*t7.*t10.*t13.*y.*5.0-t2.*t10.*t11.*t18.*t24.*t29.*t33.*t34.*x.*5.0+t2.*t10.*t24.*t29.*t32.*t33.*t34.*x.*z.*5.0,t63+t2.*t6.*t7.*t10.*t13.*x.*5.0-t2.*t7.*t10.*t13.*t18.*y.*5.0-t2.*t10.*t11.*t18.*t24.*t29.*t33.*t34.*y.*5.0,t24.*t32.*t34.*t68.*x.*5.0-t18.*t24.*t34.*t68.*x.*z.*5.0,0.0,1.0,0.0,t63-t3.*t6.*t7.*t10.*t13.*x.*5.0-t2.*t7.*t10.*t13.*t18.*y.*5.0-t2.*t10.*t11.*t18.*t24.*t29.*t33.*t34.*y.*5.0,t2.*t7.*t10.*t13.*t18.*x.*5.0+t2.*t6.*t7.*t10.*t13.*y.*5.0-t3.*t10.*t11.*t18.*t24.*t29.*t33.*t34.*x.*5.0+t3.*t10.*t24.*t29.*t32.*t33.*t34.*x.*z.*5.0,t24.*t32.*t34.*t68.*y.*5.0-t18.*t24.*t34.*t68.*y.*z.*5.0,0.0,0.0,1.0,t2.*t10.*t18.*t24.*t29.*t34.*z.*5.0-t6.*t7.*t10.*t13.*x.*y.*z.*5.0+t2.*t10.*t11.*t24.*t29.*t32.*t33.*t34.*5.0,t2.*t6.*t7.*t10.*t13.*z.*5.0+t10.*t18.*t24.*t29.*t34.*x.*y.*z.*5.0+t10.*t11.*t24.*t29.*t32.*t33.*t34.*x.*y.*5.0,t4.*t18.*t24.*t34.*t68.*5.0+t24.*t32.*t34.*t68.*z.*5.0],[6,6]);