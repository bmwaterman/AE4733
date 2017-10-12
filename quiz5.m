

% We scaled the m to km to fix bad scaling
R = diag([0.25 0.1].^2)

z = [4.68 1.0272]'
C = [180/(12*pi), 1]'

xi_hat = inv(C' * inv(R) * C ) * C' * inv(R) * z % km