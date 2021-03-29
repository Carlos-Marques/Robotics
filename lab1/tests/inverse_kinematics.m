%% Inverse Kinematics
clear

cos_s = sym('c', [1 7]);
sin_s = sym('s', [1 7]);
r_s = sym('r', [1 7]);
d_s = sym('d', [1 7]);
alpha = [0 90 0 90 -90 90 0];

r_s(1) = 0;
r_s(2) = 0;
r_s(5) = 0;
r_s(7) = 0;

d_s(3) = 0;
d_s(4) = 0;
d_s(5) = 0;
d_s(6) = 0;

cos_s(1) = 1;
sin_s(1) = 0;

% Transformations
for i=1:7
   T(:,:,i) = [cos_s(i) -sin_s(i)*cosd(alpha(i)) sin_s(i)*sind(alpha(i)) r_s(i)*cos_s(i); 
               sin_s(i) cos_s(i)*cosd(alpha(i)) -cos_s(i)*sind(alpha(i)) r_s(i)*sin_s(i); 
               0             sind(alpha(i))                cosd(alpha(i))               d_s(i); 
               0             0                            0                           1];
end

syms r11 r12 r13 r21 r22 r23 r31 r32 r33 px py pz

Tbase = [r11 r12 r13 px;
r21 r22 r23 py;
r31 r32 r33 pz;
0 0 0 1];