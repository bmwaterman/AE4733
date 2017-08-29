TAS = 70; %m/s
g = 10; %m/s^2
bank = 20*pi/180; %radians
syms r;
turn_radius = vpasolve(bank-atan(((TAS^2)/r)/g),r,1000); % m
time = vpa(((90*pi/180)*turn_radius)/TAS); % s

