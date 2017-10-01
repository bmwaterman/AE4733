theta = sym('theta');
alpha = sym('alpha');
psi = sym('psi');
beta = sym('beta')

equation = [cos(theta) = cos(alpha);cos(psi)=cos(alpha)*cos(beta); cos(psi)*cos(theta) = cos(beta);]
solve(equation)

