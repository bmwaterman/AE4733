## Homework 1 Extra Credit
# Bailey Waterman and Keshuai Xu

# References
# Hofmann-Wellenhof, Bernhard, and Helmut Moritz. Physical geodesy. Springer Science & Business Media, 2006.

a = 1;
b = 1;

N = @(phi) a^2 * (a^2*cos(phi)^2 + b^2*sin(phi)^2))^(-0.5)
geodesic2ctrs = @(lat, long, h) [(N(lat)+h)*cos(lat)*cos(long);
                                (N(lat)+h)*cos(lat)*sin(long);
                                (b^2*N(lat)/a^2+h)*sin(lat)];
