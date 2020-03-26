%% Last 3 joints

% T_0_4 has been calculated 

% T_base_end is built from the input

alpha = 0;
beta = -1.5708;
gama = 0;
x = (d4 + d5);
y = 0;
z = d0 + d1 + d2 +d3;

rot_base_end = eul2rotm([alpha beta gama], 'ZYZ');

T_base_end = [rot_base_end [x y z]'; 0 0 0 1];

% Inverse of T_0_4 - [R' | -R'p]
inv_T_0_4 = [T_0_4(1:3,1:3)' -T_0_4(1:3,1:3)'*T_0_4(1:3,4); 0 0 0 1];

% Left side of algebric method equation
first_joints = inv_T_0_4*T_base_end;

theta5 = acos(first_joints(3,3));

theta6 = asin(first_joints(3,2)/sin(theta5));

theta4 = asin(first_joints(2,3)/sin(theta5));
