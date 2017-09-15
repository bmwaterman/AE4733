function [receiver_position, epsilon_c] = trilat_clockbias(satellite_positions, measured_ranges, initial_guess, e1, e2, max_iter)
%TRILAT_CLOCKBIAS Trilateration for radio ranging

big_fat_one = @(p_r_guess, p_sat) (p_r_guess - p_sat) / norm(p_r_guess - p_sat);
receiver_position = initial_guess;
epsilon_c = 0;

for iter = 1:max_iter
    A = cell2mat(cellfun(@(p_sat) [big_fat_one(receiver_position, p_sat); 1]', num2cell(satellite_positions, 1), 'UniformOutput', false)');
    b = measured_ranges - ones(size(measured_ranges,1),1)*epsilon_c - cellfun(@norm, num2cell(bsxfun(@minus, receiver_position, satellite_positions), 1))';
    d_p = A \ b;
    receiver_position = receiver_position + d_p(1:end-1, :);
    epsilon_c = epsilon_c + d_p(end, :);
    if norm(d_p) < e1 || norm(b) < e2
        return
    end
end
warning('reached maximum iteration')
b
end