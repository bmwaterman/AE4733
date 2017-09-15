function [ a_t ] = hw2_a_t( t )
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
if t < 0; error('out of bound'); end;
if t < 10; a_t = 0; return; end;
if t < 25; a_t = 5; return; end;
if t < 35; a_t = 5; return; end;
if t < 40; a_t = 0; return; end;
error('out of bound');

end

