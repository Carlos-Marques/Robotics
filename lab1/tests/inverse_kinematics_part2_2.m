clear

%% First 3 joints

% T_0_4 has been calculated 

% T_base_end is built from the input

alpha = deg2rad(0);
beta = deg2rad(-90);
gama = deg2rad(180);
x = 0.3;
y = 0;
z = 0.7;

rot_base_end = eul2rotm([alpha beta gama], 'ZYZ');

T_base_end = [rot_base_end [x y z]'; 0 0 0 1];

p_0_5 = [x y z]' - rot_base_end(:,end)*0.0243;

theta1 = atan2(p_0_5(2), p_0_5(1));

p_2_5 = p_0_5 - [0 0 0.183]';

l2 = 0.21;

l4 = sqrt(0.03^2+0.2215^2);

phi = acos((l2^2+l4^2-sum(abs(p_2_5).^2))/2^l2^l4);

theta3 = 3/2*pi-phi;

T_0_2 = [cos(theta1) 0 sin(theta1) 0; 
         sin(theta1) 0 -cos(theta1) 0;
         0 1 0 0.183;
         0 0 0 1];
     
p2_2_5 = T_0_2(1:3,1:3)*p_2_5;

beta1 = atan2(-p2_2_5(2), p2_2_5(1));
beta2 = acos((l2^2-l4^2+sum(abs(p_2_5).^2))/2*l2*sqrt(sum(abs(p_2_5).^2)));

theta2 = -(beta1+beta2);

T_2_3 = [cos(theta2) sin(theta2) 0 -0.21*cos(theta2);
         -sin(theta2) cos(theta2) 0 0.21*sin(theta2);
         0 0 1 0;
         0 0 0 1];
T_3_4 = [cos(theta3) 0 sin(theta3) -0.03*cos(theta3);
         sin(theta3) 0 -cos(theta3) -0.03*sin(theta3);
         0 1 0 0;
         0 0 0 1];
  
T_0_4 = T_0_2*T_2_3*T_3_4;
%% Last 3 joints

% Inverse of T_0_4 - [R' | -R'p]
inv_T_0_4 = [T_0_4(1:3,1:3)' -T_0_4(1:3,1:3)'*T_0_4(1:3,4); 0 0 0 1];

% Left side of algebric method equation
first_joints = inv_T_0_4*T_base_end;

theta5 = acos(first_joints(3,3));

theta6 = asin(first_joints(3,2)/sin(theta5));

theta4 = asin(first_joints(2,3)/sin(theta5));
