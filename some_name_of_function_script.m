% define initial conditions, 
% load given measurements

% use the same variable for timespan as given in the data

%run ode_45
ode45(@(t,y) some_name_of_function(t,y, whatever_else), timespan, initial_conditions)