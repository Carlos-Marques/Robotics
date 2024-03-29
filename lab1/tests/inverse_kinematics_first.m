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
r2 = Z -(d1+d0);
r3 = d4 + d5;
r4 = sqrt(r1^2 + r2^2);
r5 = sqrt(r3^2 + d3^2);

phi1 = atan2(r2, r1);
phi2 = atan2(r3, d3);
phi3 = acos((r4^2 - d2^2 - r5^2) / (-2 * d2 * r5));
phi4 = acos((r5^2 - d2^2 - r4^2) / (-2 * d2 * r4));

tr = phi1 + phi4;

A1 = rad2deg(atan2(Y, X));
A2 = -(90 - rad2deg(tr));
A3 = -( 180 - rad2deg(phi2) - rad2deg(phi3));

phi22 = atan2(r3, d3);
phi32 = -acos((r4^2 - d2^2 - r5^2) / (-2 * d2 * r5));
A12 = rad2deg(atan2(Y, X))+180;
A22 = -A2;
A32 = -( 180 - rad2deg(phi22) - rad2deg(phi32));

phi23 = atan2(r3, d3);
phi33 = acos((r4^2 - d2^2 - r5^2) / (-2 * d2 * r5));
phi43 = -acos((r5^2 - d2^2 - r4^2) / (-2 * d2 * r4));
tr3 = phi1 + phi43;

A13 = rad2deg(atan2(Y, X))+180;
A23 = (90 - rad2deg(tr3));
A33 = -( 180 - rad2deg(phi23) - rad2deg(phi33));


A14 = A1;
phi34 = -acos((r4^2 - d2^2 - r5^2) / (-2 * d2 * r5));
phi44 = -acos((r5^2 - d2^2 - r4^2) / (-2 * d2 * r4));
tr4 = phi1 + phi44;
A24 = -(90 - rad2deg(tr4));
A34 = -( 180 - rad2deg(phi2) - rad2deg(phi34));