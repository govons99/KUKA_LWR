clc
clear
close all

%% Symbolic variables

syms q1 q2 q3 q4 q5 q6 q7 l1 l2 l3 l4

q = [q1;q2;q3;q4;q5;q6;q7];

%% Direct kinematics

C1=cos(q1);
S1=sin(q1);
C2=cos(q2);
S2=sin(q2);
C3=cos(q3);
S3=sin(q3);
C4=cos(q4);
S4=sin(q4);
C5=cos(q5);
S5=sin(q5);
C6=cos(q6);
S6=sin(q6);
C7=cos(0.0);
S7=sin(0.0);

% l1=0.0;     % for real robot where the base frame is on the second joint
% l2=0.4;
% l3=0.39;
% l4=0.078;   % EE in the tip of KUKA without auxiliary addition

M1 = [C1,0,S1,0; S1,0,-C1,0; 0,1,0,l1; 0,0,0,1];
M2 = [C2,0,-S2,0; S2,0,C2,0; 0,-1,0,0; 0,0,0,1];
M3 = [C3,0,-S3,0; S3,0,C3,0; 0,-1,0,l2; 0,0,0,1];
M4 = [C4,0,S4,0; S4,0,-C4,0; 0,1,0,0; 0,0,0,1];
M5 = [C5,0,S5,0; S5,0,-C5,0; 0,1,0,l3; 0,0,0,1];
M6 = [C6,0,-S6,0; S6,0,C6,0; 0,-1,0,0; 0,0,0,1];
M7 = [C7,-S7,0,0; S7,C7,0,0; 0,0,1,l4; 0,0,0,1];

A7 = (((((M1*M2)*M3)*M4)*M5)*M6)*M7;
A6 = ((((M1*M2)*M3)*M4)*M5)*M6;
A5 = (((M1*M2)*M3)*M4)*M5;
A4 = ((M1*M2)*M3)*M4;
A3 = (M1*M2)*M3;
A2 = M1*M2;
A1 = M1;

p7 = simplify(A7(1:3,4));
p6 = simplify(A6(1:3,4));
p5 = simplify(A5(1:3,4));
p4 = simplify(A4(1:3,4));
p3 = simplify(A3(1:3,4));
p2 = simplify(A2(1:3,4));
p1 = simplify(A1(1:3,4));

JEE = jacobian(p7,q);
J6 = jacobian(p6,q);
J5 = jacobian(p5,q);
J4 = jacobian(p4,q);
J3 = jacobian(p3,q);
J2 = jacobian(p2,q);
J1 = jacobian(p1,q);











