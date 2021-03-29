%% Forward kinematics

% Mechanical specifications
d0 = 0.103;
d1 = 0.080;
d2 = 0.210;
d3 = 0.030;
d4 = 0.0415;
d5 = 0.180;
d6 = 0;
d7 = 0.0237;

% input
X = final(1, 4);
Y = final(2, 4);
Z = final(3, 4);

% Full transform built from the input
rot_base_end = final(1:3, 1:3);
T_base_end = [rot_base_end [X Y Z]'; 0 0 0 1];

% Transform xyz to the frame in 5
p = final(1:3, 4);
p_5 = p + final(1:3,3)*d7;
X = p_5(1);
Y = p_5(2);
Z = p_5(3);

% Theta1,2,3 calculation
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
%% Last 3 joints

A1 = deg2rad(A1);
A2 = deg2rad(A2);
A3 = deg2rad(A3);
% Forward kinematics to get T_0_4
% DH parameters
theta = [0 A1 A2-deg2rad(90) A3];
alpha = [0 deg2rad(90) 0 deg2rad(90)];
r = [0 0 -d2 -d3];
d = [d0 d1 0 0];

T = zeros(4,4, length(theta));
% Transformations
for i=1:length(theta)
   T(:,:,i) = [cos(theta(i)) -sin(theta(i))*cos(alpha(i)) sin(theta(i))*sin(alpha(i)) r(i)*cos(theta(i)); 
               sin(theta(i)) cos(theta(i))*cos(alpha(i)) -cos(theta(i))*sin(alpha(i)) r(i)*sin(theta(i)); 
               0 sin(alpha(i)) cos(alpha(i)) d(i); 
               0 0 0 1];
end

T_0_4 = T(:, :, 1)*T(:, :, 2)*T(:, :, 3)*T(:, :, 4);

inv_T_0_4 = [T_0_4(1:3,1:3)' -T_0_4(1:3,1:3)'*T_0_4(1:3,4); 0 0 0 1];

% Left side of algebric method equation
first_joints = inv_T_0_4*T_base_end;

A4 = atan2(first_joints(2,3),first_joints(1,3));

A5 = acos(first_joints(3,3));

A6 = atan2(first_joints(3,2),-first_joints(3,1));

% DH parameters
theta = [0 A1 A2-deg2rad(90) A3 A4 A5 A6];
alpha = [0 deg2rad(90) 0 deg2rad(90) deg2rad(-90) deg2rad(90) 0];
r = [0 0 -d2 -d3 0 d6 0];
d = [d0 d1 0 0 -(d4+d5) 0 -d7];

T = zeros(4,4, length(theta));
% Transformations
for i=1:length(theta)
   T(:,:,i) = [cos(theta(i)) -sin(theta(i))*cos(alpha(i)) sin(theta(i))*sin(alpha(i)) r(i)*cos(theta(i)); 
               sin(theta(i)) cos(theta(i))*cos(alpha(i)) -cos(theta(i))*sin(alpha(i)) r(i)*sin(theta(i)); 
               0             sin(alpha(i))                cos(alpha(i))               d(i); 
               0             0                            0                           1];
end

final_2 = T(:, :, 1)*T(:, :, 2)*T(:, :, 3)*T(:, :, 4)*T(:, :, 5)*T(:, :, 6)*T(:, :, 7)

A1 = rad2deg(A1);
A2 = rad2deg(A2);
A3 = rad2deg(A3);
A4 = rad2deg(A4);
A5 = rad2deg(A5);
A6 = rad2deg(A6);
