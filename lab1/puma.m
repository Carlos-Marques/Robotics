clear
syms r11 r12 r13 r21 r22 r23 r31 r32 r33 px py pz
syms c1 s1 c2 s2 c3 s3 a2 d3 c4 s4 a3 d4 s5 c5 c6 s6

T0_1 = [c1 -s1 0 0;
s1 c1 0 0
0 0 1 0
0 0 0 1];

T1_2 = [c2 -s2 0 0;
0 0 1 0;
-s2 -c2 0 0;
0 0 0 1];

T2_3 = [c3 -s3 0 a2;
s3 c3 0 0;
0 0 1 d3;
0 0 0 1];

T3_4 = [c4 -s4 0 a3;
0 0 1 d4;
-s4 -c4 0 0;
0 0 0 1];

T4_5 = [c4 -s5 0 0;
0 0 -1 0;
s5 c5 0 0;
0 0 0 1];

T5_6 = [c6 -s6 0 0;
0 0 1 0;
-s6 -c6 0 0;
0 0 0 1];

T1_6 = T1_2*T2_3*T3_4*T4_5*T5_6;

test = [ c1 s1 0 0;
-s1 c1 0 0;
0 0 1 0;
0 0 0 1];

testb = [r11 r12 r13 px;
r21 r22 r23 py;
r31 r32 r33 pz;
0 0 0 1];

dir = test*testb;