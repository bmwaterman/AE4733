function [ a_l ] = hw2_a_l( t )
%HW2_A_L Summary of this function goes here
%   Detailed explanation goes here
if t < 0; a_l = 5; warning('out of bound'); end;
if t < 10; a_l = 5; return; end;
if t < 25; a_l = 0; return; end;
if t < 35; a_l = 5; return; end;
if t < 40; a_l = 0; return; end;
a_l = 0;
warning('out of bound');

end

