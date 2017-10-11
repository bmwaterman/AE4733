function [ output_args ] = Rte(yaw_321, pitch_321, roll_321)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

output_args = ...
    [1 0 0;
    0 cos(roll_321) sin(roll_321);
    0 -sin(roll_321) cos(roll_321)] * ...
    [cos(pitch_321) 0 -sin(pitch_321);
    0 1 0;
    sin(pitch_321) 0 cos(pitch_321)] * ...
    [cos(yaw_321) sin(yaw_321) 0;
    -sin(yaw_321) cos(yaw_321) 0;
    0 0 1];
end

