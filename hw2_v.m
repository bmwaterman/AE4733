function [ v ] = hw2_v( t )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

if t < 0; v = 5; warning('out of bound'); end;
if t < 10; v = 5; return; end;
if t < 25; v = 5*(t-10); return; end;
if t < 35; v = 5*(t-10); return; end;
if t < 40; v = 5*(35-10); return; end;
v = 5*(35-10);
warning('out of bound');

end

