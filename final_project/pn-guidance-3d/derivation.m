clear variable
syms t real
syms x_dot y_dot z_dot xx yy zz real
syms x(t) y(t) z(t)
alpha = atan(y / x);
beta = asin(z / sqrt(x^2 + y^2 + z^2));
alpha_dot = simplify(diff(alpha, t))
beta_dot = simplify(diff(beta, t))
v_c = simplify(- diff(sqrt(x^2 + y^2 + z^2), t))

alpha_dot_var = subs(alpha_dot, [x(t) y(t) z(t) diff(x(t), t) diff(y(t), t) diff(z(t), t)], [xx yy zz x_dot y_dot z_dot]);
beta_dot_var = subs(beta_dot, [x(t) y(t) z(t) diff(x(t), t) diff(y(t), t) diff(z(t), t)], [xx yy zz x_dot y_dot z_dot]);
v_c_var = subs(v_c, [x(t) y(t) z(t) diff(x(t), t) diff(y(t), t) diff(z(t), t)], [xx yy zz x_dot y_dot z_dot]);

in = [xx yy zz x_dot y_dot z_dot]';

alpha_dot_fun = matlabFunction(alpha_dot_var, 'Vars', {in,t})
beta_dot_fun = matlabFunction(beta_dot_var, 'Vars', {in,t})
v_c_fun = matlabFunction(v_c_var, 'Vars', {in,t})

