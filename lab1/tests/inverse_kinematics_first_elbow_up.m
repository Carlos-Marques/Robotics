%% Forward kinematics

p = final(1:3, 4);
p_5 = p + final(1:3,3)*d7;

% Mechanical specifications
d0 = 0.103;
d1 = 0.080;
d2 = 0.210;
d3 = 0.030;
d4 = 0.0415;
d5 = 0.180;
d6 = 0;
d7 = 0.0237;

X = p_5(1);
Y = p_5(2);
Z = p_5(3);

r1 = sqrt(X^2 + Y^2);
r2 = Z - d0+d1;
r3 = d4 + d5;
r4 = sqrt(r1^2 + r2^2);
r5 = sqrt(r3^2 + d3^2);

phi1 = atan2(r1, r2);
phi2 = acos((r5^2 - d2^2 -r4^2) / (-2 * r4 * d2));
phi3 = atan2(r3, d3);
phi4 = acos((r4^2 - r5^2 - d2^2) / (-2 * r5 * d2));

A1 = rad2deg(atan2(Y,X));
A2 = -(180 - rad2deg(phi1) - rad2deg(phi2));
A3 = -(180 - rad2deg(phi3) - rad2deg(phi4));