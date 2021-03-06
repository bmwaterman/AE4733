function receiver_position  = Trilateration_Matlab_Function( satellite_positions, measured_ranges )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% satellite position vectors are in columns of this variable
% "satellite_positions"

% measured ranges to satellites are in a column vector in this variable
% "measured_ranges"

% Choose initial guess
receiver_position_guess_n = [10;10;10] %this is p_r^n
eplison_c_guess = 0  

% Big while look that repeats linear solutions
epsilon_close = 0.1;
max_iterations = 100;

some_condition = true;
n_iter = 0;
while (some_condition)
    % set up matrix with big fat ones
    big_fat_one_1 = (receiver_position_guess_n - satellite_positions(:, 1)) / norm(receiver_position_guess_n - satellite_positions(:, 1)); 
    big_fat_one_2 = (receiver_position_guess_n - satellite_positions(:, 2)) / norm(receiver_position_guess_n - satellite_positions(:, 2));
     big_fat_one_3 = (receiver_position_guess_n - satellite_positions(:, 3)) / norm(receiver_position_guess_n - satellite_positions(:, 3));
    A = [big_fat_one_1'; big_fat_one_2'; big_fat_one_3'] 
    b = measured_ranges - [norm(receiver_position_guess_n - satellite_positions(:, 1));...
        norm(receiver_position_guess_n - satellite_positions(:, 2));...
        norm(receiver_position_guess_n - satellite_positions(:, 3))]
   
    % solve linear equation
    % delta_p = A^(-1) * b
    delta_p = A \ b
    
    % update guess of reciever position
    receiver_position_guess_n = receiver_position_guess_n + delta_p;
    n_iter = n_iter + 1
    
    %Conditions for stopping iterations
    % 1. Close to the actual solution (norm values are close to measured
    % values)
    some_condition = ~ (norm(measured_ranges - [norm(receiver_position_guess_n - satellite_positions(:, 1));...
        norm(receiver_position_guess_n - satellite_positions(:, 2));...
        norm(receiver_position_guess_n - satellite_positions(:, 3))])) < epsilon_close && ~ (n_iter >= max_iterations);
    
    % 2. Exceeded max number of allowed iterations
   
    
    % 3. improvements (delta_p) become too small
    
end 
receiver_position = receiver_position_guess_n;

end

