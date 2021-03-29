syms t1 t2 t3 t4 t5 t6

d0 = 0.103;
d1 = 0.080;
d2 = 0.210;
d3 = 0.030;
d4 = 0.0415;
d5 = 0.180;
d6 = 0.0055;
d7 = 0.0237;

T_01 = [1 0 0 0; 0 1 0 0; 0 0 1 d0;0 0 0 1];

T_12 = [cos(t1) 0 sin(t1) 0; sin(t1) 0 -cos(t1) 0; 0 1 0 d1; 0 0 0 1];

T_23 = [cos(t2) sin(t2) 0 -d2*cos(t2);-sin(t2) cos(t2) 0 d2*sin(t2); 0 0 1 0; 0 0 0 1];

T_34 = [cos(t3) 0 sin(t3) -d3*cos(t3); sin(t3) 0 -cos(t3) -d3*sin(t3); 0 1 0 0; 0 0 0 1];

T_45 = [cos(t4) 0 -sin(t4) 0; sin(t4) 0 cos(t4) 0; 0 -1 0 -d4-d5; 0 0 0 1];

T_56 = [cos(t5) 0 sin(t5) -d6*cos(t5); sin(t5) 0 -cos(t5) -d6*sin(t5); 0 1 0 0; 0 0 0 1];

T_67 = [cos(t6) -sin(t6) 0 0; sin(t6) cos(t6) 0 0; 0 0 1 -d7;0 0 0 1];

T_07 = T_01*T_12*T_23*T_34*T_45*T_56*T_67;

T_04 = T_01*T_12*T_23*T_34;

J_04 = jacobian(T_04(1:3,4), [t1 t2 t3]);

J_07 = jacobian(T_07(1:3,4), [t1 t2 t3 t4 t5 t6]);

J_07_t = J_07';

det_jacobian = det(J_04);

det_j1 = det( [J_07_t(4,:) ; J_07_t(5,:) ; J_07_t(1,:) ] ) == 0;

det_j2 = det( [J_07_t(4,:) ; J_07_t(5,:) ; J_07_t(2,:) ] ) == 0;

det_j3 = det( [J_07_t(4,:) ; J_07_t(5,:) ; J_07_t(3,:) ] ) == 0;