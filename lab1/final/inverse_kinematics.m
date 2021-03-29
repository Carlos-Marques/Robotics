%% Inverse kinematics
% Inputs: A1, A2, A3 - x, y, z of end effector in mm
%         A4, A5, A6 - alpha, beta, gama of end effector in rad (ZYZ)
% Returns: matrix (number_solutions x 6) with joint values

function solution_matrix = inverse_kinematics(A1, A2, A3, A4, A5, A6)

% Mechanical specifications
% We ignore the 5.5mm at the end effector
d0 = 0.103;
d1 = 0.080;
d2 = 0.210;
d3 = 0.030;
d4 = 0.0415;
d5 = 0.180;
d6 = 0;
d7 = 0.0237;

% Store inputs in variables
px = A1;
py = A2;
pz = A3;
alpha = A4;
beta = A5;
gama = A6;

% Full transform built from the input
rot_base_end = eul2rotm([alpha beta gama], 'ZYZ');
T_base_end = [rot_base_end [px py pz]'; 0 0 0 1];

% Initialize solution matrix with empty matrix
solution_matrix = [];

% Transform px py pz to frame 5
p = [px py pz]';
p_5 = p + T_base_end(1:3,3)*d7;
px_5 = p_5(1);
py_5 = p_5(2);
pz_5 = p_5(3);

% Auxilliary calculations for first 3 joints
r1 = sqrt(px_5^2 + py_5^2);
r2 = pz_5 -(d1+d0);
r3 = d4 + d5;
r4 = sqrt(r1^2 + r2^2);
r5 = sqrt(r3^2 + d3^2);

phi1 = atan2(r2, r1);
phi2 = atan2(r3, d3);
phi3 = acos((r4^2 - d2^2 - r5^2) / (-2 * d2 * r5));
phi4 = acos((r5^2 - d2^2 - r4^2) / (-2 * d2 * r4));

tr = phi1 + phi4;

%% First solutions (1 for first 3 joints, 2 for last  - total 2 solutions)
A1 = atan2(py_5, px_5);
A2 = -(pi/2 - tr);
A3 = -( pi - phi2 - phi3);

% DH parameters
theta = [0 A1 A2-deg2rad(90) A3];
alpha_dh = [0 deg2rad(90) 0 deg2rad(90)];
r = [0 0 -d2 -d3];
d = [d0 d1 0 0];

T = zeros(4,4, length(theta));
% Transformations
for i=1:length(theta)
    T(:,:,i) = [cos(theta(i)) -sin(theta(i))*cos(alpha_dh(i)) sin(theta(i))*sin(alpha_dh(i)) r(i)*cos(theta(i));
        sin(theta(i)) cos(theta(i))*cos(alpha_dh(i)) -cos(theta(i))*sin(alpha_dh(i)) r(i)*sin(theta(i));
        0 sin(alpha_dh(i)) cos(alpha_dh(i)) d(i);
        0 0 0 1];
end

T_0_4 = T(:, :, 1)*T(:, :, 2)*T(:, :, 3)*T(:, :, 4);

inv_T_0_4 = [T_0_4(1:3,1:3)' -T_0_4(1:3,1:3)'*T_0_4(1:3,4); 0 0 0 1];

% Left side of algebric method equation
first_joints = inv_T_0_4*T_base_end;

if isreal(first_joints)
    
    A5 = acos(first_joints(3,3));
    
    A4 = atan2(first_joints(2,3),first_joints(1,3));
    
    A6 = atan2(first_joints(3,2),-first_joints(3,1));
    
    if ~isreal(A1) || (py_5==0 && px_5==0) || ~isreal(A2) || (r2==0 && r1==0) ||...
            (r3==0 && d3==0) || ~isreal(A3) || ~isreal(A4) || ~isreal(A5) ||...
            ~isreal(A6) || (first_joints(2,3)==0 && first_joints(1,3)==0) ||...
            (first_joints(3,2)==0 && -first_joints(3,1)==0)
        
        solution_matrix=[];
    else
        solution_matrix = [A1 A2 A3 A4 A5 A6];
    end
    
    % Second solution for last 3 joints
    A5_sec = -acos(first_joints(3,3));
    
    A4_sec = atan2(first_joints(2,3),first_joints(1,3))+pi;
    
    A6_sec = atan2(first_joints(3,2),-first_joints(3,1))+pi;
    
    if ~isreal(A5_sec) || ~isreal(A4_sec) || ~isreal(A6_sec) ||...
            (first_joints(2,3)==0 && first_joints(1,3)==0) ||...
            (first_joints(3,2)==0 && first_joints(3,1)==0)
    else
        solution_matrix = [solution_matrix; A1 A2 A3 A4_sec A5_sec A6_sec];
    end
end

%% Second solution
phi22 = atan2(r3, d3);
phi32 = -acos((r4^2 - d2^2 - r5^2) / (-2 * d2 * r5));
A12 = atan2(py_5, px_5)+pi;
A22 = -A2;
A32 = -( pi - phi22 - phi32);

% DH parameters
theta = [0 A12 A22-deg2rad(90) A32];
alpha_dh = [0 deg2rad(90) 0 deg2rad(90)];
r = [0 0 -d2 -d3];
d = [d0 d1 0 0];

T = zeros(4,4, length(theta));
% Transformations
for i=1:length(theta)
    T(:,:,i) = [cos(theta(i)) -sin(theta(i))*cos(alpha_dh(i)) sin(theta(i))*sin(alpha_dh(i)) r(i)*cos(theta(i));
        sin(theta(i)) cos(theta(i))*cos(alpha_dh(i)) -cos(theta(i))*sin(alpha_dh(i)) r(i)*sin(theta(i));
        0 sin(alpha_dh(i)) cos(alpha_dh(i)) d(i);
        0 0 0 1];
end

T_0_4 = T(:, :, 1)*T(:, :, 2)*T(:, :, 3)*T(:, :, 4);

inv_T_0_4 = [T_0_4(1:3,1:3)' -T_0_4(1:3,1:3)'*T_0_4(1:3,4); 0 0 0 1];

% Left side of algebric method equation
first_joints = inv_T_0_4*T_base_end;

if isreal(first_joints)
    A52 = acos(first_joints(3,3));
    
    A42 = atan2(first_joints(2,3),first_joints(1,3));
    
    A62 = atan2(first_joints(3,2),-first_joints(3,1));
    
    if ~isreal(A12) || (py_5==0 && px_5==0) || ~isreal(A22) || (r2==0 && r1==0) ||...
            (r3==0 && d3==0) || ~isreal(A32) || ~isreal(A42) || ~isreal(A52) ||...
            ~isreal(A62) || (first_joints(2,3)==0 && first_joints(1,3)==0) ||...
            (first_joints(3,2)==0 && -first_joints(3,1)==0)
        
    else
        solution_matrix = [solution_matrix; A12 A22 A32 A42 A52 A62];
    end
    
    % Second solution for last 3 joints
    A52_sec = -acos(first_joints(3,3));
    
    A42_sec = atan2(first_joints(2,3),first_joints(1,3))+pi;
    
    A62_sec = atan2(first_joints(3,2),-first_joints(3,1))+pi;
    
    if ~isreal(A52_sec) || ~isreal(A42_sec) || ~isreal(A62_sec) ||...
            (first_joints(2,3)==0 && first_joints(1,3)==0) ||...
            (first_joints(3,2)==0 && first_joints(3,1)==0)
    else
        solution_matrix = [solution_matrix; A12 A22 A32 A42_sec A52_sec A62_sec];
    end
end
%% Third solution
phi23 = atan2(r3, d3);
phi33 = acos((r4^2 - d2^2 - r5^2) / (-2 * d2 * r5));
phi43 = -acos((r5^2 - d2^2 - r4^2) / (-2 * d2 * r4));
tr3 = phi1 + phi43;

A13 = atan2(py_5, px_5)+pi;
A23 = (pi/2 - (tr3));
A33 = -( pi - (phi23) - (phi33));

% DH parameters
theta = [0 A13 A23-deg2rad(90) A33];
alpha_dh = [0 deg2rad(90) 0 deg2rad(90)];
r = [0 0 -d2 -d3];
d = [d0 d1 0 0];

T = zeros(4,4, length(theta));
% Transformations
for i=1:length(theta)
    T(:,:,i) = [cos(theta(i)) -sin(theta(i))*cos(alpha_dh(i)) sin(theta(i))*sin(alpha_dh(i)) r(i)*cos(theta(i));
        sin(theta(i)) cos(theta(i))*cos(alpha_dh(i)) -cos(theta(i))*sin(alpha_dh(i)) r(i)*sin(theta(i));
        0 sin(alpha_dh(i)) cos(alpha_dh(i)) d(i);
        0 0 0 1];
end

T_0_4 = T(:, :, 1)*T(:, :, 2)*T(:, :, 3)*T(:, :, 4);

inv_T_0_4 = [T_0_4(1:3,1:3)' -T_0_4(1:3,1:3)'*T_0_4(1:3,4); 0 0 0 1];

% Left side of algebric method equation
first_joints = inv_T_0_4*T_base_end;

if isreal(first_joints)
    A53 = acos(first_joints(3,3));
    
    A43 = atan2(first_joints(2,3),first_joints(1,3));
    
    A63 = atan2(first_joints(3,2),-first_joints(3,1));
    
    if ~isreal(A13) || (py_5==0 && px_5==0) || ~isreal(A23) || (r2==0 && r1==0) ||...
            (r3==0 && d3==0) || ~isreal(A33) || ~isreal(A43) || ~isreal(A53) ||...
            ~isreal(A63) || (first_joints(2,3)==0 && first_joints(1,3)==0) ||...
            (first_joints(3,2)==0 && -first_joints(3,1)==0)
        
    else
        solution_matrix = [solution_matrix; A13 A23 A33 A43 A53 A63];
    end
    
    % Second solution for last 3 joints
    A53_sec = -acos(first_joints(3,3));
    
    A43_sec = atan2(first_joints(2,3),first_joints(1,3))+pi;
    
    A63_sec = atan2(first_joints(3,2),-first_joints(3,1))+pi;
    
    if ~isreal(A53_sec) || ~isreal(A43_sec) || ~isreal(A63_sec) ||...
            (first_joints(2,3)==0 && first_joints(1,3)==0) ||...
            (first_joints(3,2)==0 && first_joints(3,1)==0)
    else
        solution_matrix = [solution_matrix; A13 A23 A33 A43_sec A53_sec A63_sec];
    end
end
%% Fourth solution
A14 = A1;
phi34 = -acos((r4^2 - d2^2 - r5^2) / (-2 * d2 * r5));
phi44 = -acos((r5^2 - d2^2 - r4^2) / (-2 * d2 * r4));
tr4 = phi1 + phi44;
A24 = -(pi/2 - (tr4));
A34 = -( pi - (phi2) - (phi34));

% DH parameters
theta = [0 A14 A24-deg2rad(90) A34];
alpha_dh = [0 deg2rad(90) 0 deg2rad(90)];
r = [0 0 -d2 -d3];
d = [d0 d1 0 0];

T = zeros(4,4, length(theta));
% Transformations
for i=1:length(theta)
    T(:,:,i) = [cos(theta(i)) -sin(theta(i))*cos(alpha_dh(i)) sin(theta(i))*sin(alpha_dh(i)) r(i)*cos(theta(i));
        sin(theta(i)) cos(theta(i))*cos(alpha_dh(i)) -cos(theta(i))*sin(alpha_dh(i)) r(i)*sin(theta(i));
        0 sin(alpha_dh(i)) cos(alpha_dh(i)) d(i);
        0 0 0 1];
end

T_0_4 = T(:, :, 1)*T(:, :, 2)*T(:, :, 3)*T(:, :, 4);

inv_T_0_4 = [T_0_4(1:3,1:3)' -T_0_4(1:3,1:3)'*T_0_4(1:3,4); 0 0 0 1];

% Left side of algebric method equation
first_joints = inv_T_0_4*T_base_end;

if isreal(first_joints)
    A54 = acos(first_joints(3,3));
    
    A44 = atan2(first_joints(2,3),first_joints(1,3));
    
    A64 = atan2(first_joints(3,2),-first_joints(3,1));
    
    if ~isreal(A14) || (py_5==0 && px_5==0) || ~isreal(A24) || (r2==0 && r1==0) ||...
            (r3==0 && d3==0) || ~isreal(A34) || ~isreal(A44) || ~isreal(A54) ||...
            ~isreal(A64) || (first_joints(2,3)==0 && first_joints(1,3)==0) ||...
            (first_joints(3,2)==0 && -first_joints(3,1)==0)
        
    else
        solution_matrix = [solution_matrix; A14 A24 A34 A44 A54 A64];
    end
    
    % Second solution for last 3 joints
    A54_sec = -acos(first_joints(3,3));
    
    A44_sec = atan2(first_joints(2,3),first_joints(1,3))+pi;
    
    A64_sec = atan2(first_joints(3,2),-first_joints(3,1))+pi;
    
    if ~isreal(A53_sec) || ~isreal(A43_sec) || ~isreal(A63_sec) ||...
            (first_joints(2,3)==0 && first_joints(1,3)==0) ||...
            (first_joints(3,2)==0 && first_joints(3,1)==0)
    else
        solution_matrix = [solution_matrix; A14 A24 A34 A44_sec A54_sec A64_sec];
    end
end
if isempty(solution_matrix)
    disp('No solution');
end

end