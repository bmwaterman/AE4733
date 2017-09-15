function [ v ] = hw2_v( t )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

if t < 0; error('out of bound'); end;
if t < 10; v = 5; return; end;
if t < 25; v = 5*(t-10); return; end;
if t < 35; v = 5*(t-10); return; end;
if t < 40; v = 5*(35-10); return; end;
error('out of bound');

end

