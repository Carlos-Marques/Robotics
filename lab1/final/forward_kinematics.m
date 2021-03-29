%% Forward kinematics
% Inputs: A1, A2, A3, A4, A5, A6 - joint angles
% Returns: x,y,z,alpha,beta,gama of end effector (ZYZ convention)

function end_effector_pose = forward_kinematics(A1, A2, A3, A4, A5, A6)

% Mechanical specifications
d0 = 0.103;
d1 = 0.080;
d2 = 0.210;
d3 = 0.030;
d4 = 0.0415;
d5 = 0.180;
d6 = 0;%0.0055;
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

final = T(:, :, 1)*T(:, :, 2)*T(:, :, 3)*T(:, :, 4)*T(:, :, 5)*T(:, :, 6)*T(:, :, 7);

end_effector_pose = [final(1:3, end)' rotm2eul(final(1:3,1:3), 'ZYZ')];

end