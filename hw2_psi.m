function [ psi ] = hw2_psi( t )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
if t < 0; psi = t; warning('out of bound'); end;
if t < 10; psi = t; return; end;
if t < 25; psi = 0; return; end;
if t < 35; psi = log(t-10); return; end;
if t < 40; psi = 0; return; end;
psi = 0;
warning('out of bound');

end

