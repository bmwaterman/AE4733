function [ psi_dot ] = hw2_psi_dot( t )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
if t < 0; error('out of bound'); end;
if t < 10; psi_dot = 1; return; end;
if t < 25; psi_dot = 0; return; end;
if t < 35; psi_dot = 1/(t-10); return; end;
if t < 40; psi_dot = 0; return; end;
error('out of bound');

end

