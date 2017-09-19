%% Homework 1 Extra Credit
% Bailey Waterman and Keshuai Xu
%
% <<IMG_0918.JPG>>
%
%
% Reference:
% Hofmann-Wellenhof, Bernhard, and Helmut Moritz. Physical geodesy. Springer Science & Business Media, 2006.

clear variables; close all; clc;

% WGS84
% http://earth-info.nga.mil/GandG/publications/tr8350.2/wgs84fin.pdf
a = 6378137.0; % m
inv_f = 298.257223563;
b = a*(1-1/inv_f); % m

N = @(phi) a^2 * (a^2*cos(phi)^2 + b^2*sin(phi)^2)^(-0.5);

% input in radians and m, output in m
geodesic2ctrs = @(lat, long, h) [(N(lat)+h)*cos(lat)*cos(long);
                                (N(lat)+h)*cos(lat)*sin(long);
                                (b^2*N(lat)/a^2+h)*sin(lat)];

sole_ctrs = geodesic2ctrs(deg2rad(42.271167),deg2rad(-71.807627),0) % m