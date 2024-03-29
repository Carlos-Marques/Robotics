%% Forward kinematics
clear

% Joint values
A1 = deg2rad(0);
A2 = deg2rad(0);
A3 = deg2rad(0);
A4 = deg2rad(0);
A5 = deg2rad(0);
A6 = deg2rad(0);

% Mechanical specifications
d0 = 0.103;
d1 = 0.080;
d2 = 0.210;
d3 = 0.030;
d4 = 0.0415;
d5 = 0.180;
d6 = 0;
d7 = 0.0237;

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

final = T(:, :, 1)*T(:, :, 2)*T(:, :, 3)*T(:, :, 4)*T(:, :, 5)*T(:, :, 6)*T(:, :, 7)


p = final(1:3, 4);
p_5 = p + final(1:3,3)*d7