function J = jacobiangeometric2(theta)
% 求几何雅克比矩阵，没有工具坐标系

% 
a1 = 400;
d0 = 830;
a2 = 1175;
a3 = 250;
d4 = 1125.33;
d6 = 230;

alpha0 = 180 / 180 * pi;
alpha1 = 90 / 180 * pi;
alpha2 = 180 / 180 * pi;
alpha3 = -90 / 180 * pi;
alpha4 = -90 / 180 * pi;
alpha5 = 90 / 180 * pi;
alpha6 = 180 / 180 * pi;

% w1 = [0 0 -1];
% w2 = [0 1 0];
% w3 = [0 -1 0];
% w4 = [0 0 -1];
% w5 = [0 1 0];
% w6 = [0 0 -1];
w1 = [0 0 1];
w2 = [0 0 1];
w3 = [0 0 1];
w4 = [0 0 1];
w5 = [0 0 1];
w6 = [0 0 1];

Ab = DHmatrix(0, d0, 0, alpha0); 
A1 = DHmatrix(theta(1), 0, a1, alpha1);
A2 = DHmatrix(theta(2) - pi / 2, 0, a2, alpha2);
A3 = DHmatrix(theta(3) + theta(2) + pi / 2, 0, a3, alpha3);
A4 = DHmatrix(theta(4), -d4, 0, alpha4);
A5 = DHmatrix(theta(5), 0, 0, alpha5);
A6 = DHmatrix(theta(6) + pi, -d6, 0, alpha6);

Ttemp = [1, 0, 0, 0;
         0, cos(pi), -sin(pi), 0;
         0, sin(pi), cos(pi), 0;
         0, 0, 0, 1];

w1b = Ab * [w1, 0]';
w2b = Ab * A1 * [0, 0, 0, 0]';
w3b = Ab * A1 * Ttemp * [w3, 0]';
w4b = Ab * A1 * A2 * A3 * [w4, 0]';
w5b = Ab * A1 * A2 * A3 * A4 * [w5, 0]';
w6b = Ab * A1 * A2 * A3 * A4 * A5 * [w6, 0]';

c1 = cos(theta(1));
s1 = sin(theta(1));
c2 = cos(theta(2));
s2 = sin(theta(2));
c3 = cos(theta(3));
s3 = sin(theta(3));
c4 = cos(theta(4));
s4 = sin(theta(4));
c5 = cos(theta(5));
s5 = sin(theta(5));

J = zeros(6, 6);

J(1, 1) = s1 *( -a1  -s2 * a2 + c3 * a3 + s3 * d4 + c5 * s3 * d6 - c3 * c4 * s5 * d6) - c1 * s4 * s5 * d6;
J(1, 2) = c1 * c2 * a2;
J(1, 3) = c1 * (s3 * a3 - c3 * d4 - c3 * c5 * d6 - c4 * s3 * s5 * d6);
J(1, 4) = -c4 * s1 * s5 * d6 - c1 * c3 * s4 * s5 * d6;
J(1, 5) = c1 * c3 * c4 * c5 * d6 - c5 * s1 * s4 * d6 + c1 * s3 * s5 * d6;
J(1, 6) = 0;

J(2, 1) = c1 * (-a1 - s2 * a2 + c3 * a3 + s3 * d4 + c5 * s3 * d6 - c3 * c4 * s5 * d6) + s1 * s4 * s5 * d6;
J(2, 2) = -c2 * s1 * a2;
J(2, 3) = s1 * (-s3 * a3 + c3 * d4 + c3 * c5 * d6 + c4 * s3 * s5 * d6);
J(2, 4) = -c1 * c4 * s5 * d6 + c3 * s1 * s4 * s5 * d6;
J(2, 5) = -s1 * (-c3 * c4 * c5 * d6 - s3 * s5 * d6) - c1 * c5 * s4 * d6;
J(2, 6) = 0;

J(3, 1) = 0;
J(3, 2) = -s2 * a2;
J(3, 3) = -c3 * a3 - s3 * d4 - c5 * s3 * d6 + c3 * c4 * s5 * d6;
J(3, 4) = -s3 * s4 * s5 * d6;
J(3, 5) = c4 * c5 * s3 * d6 - c3 * s5 * d6;
J(3, 6) = 0;

J(4 : 6, 1) = w1b(1:3);
J(4 : 6, 2) = w2b(1:3);
J(4 : 6, 3) = w3b(1:3);
J(4 : 6, 4) = w4b(1:3);
J(4 : 6, 5) = w5b(1:3);
J(4 : 6, 6) = w6b(1:3);
% xi1b = [cross(w1b(1:3), pe(1 : 3) - p1b(1 : 3)); w1b(1:3)];
% xi2b = [cross(w2b(1:3), pe(1 : 3) - p2b(1 : 3)); w2b(1:3)];
% xi3b = [cross(w3b(1:3), pe(1 : 3) - p3b(1 : 3)); w3b(1:3)];
% xi4b = [cross(w4b(1:3), pe(1 : 3) - p4b(1 : 3)); w4b(1:3)];
% xi5b = [cross(w5b(1:3), pe(1 : 3) - p5b(1 : 3)); w5b(1:3)];
% xi6b = [cross(w6b(1:3), pe(1 : 3) - p6b(1 : 3)); w6b(1:3)];
% 
% 
% J = [xi1b, xi2b, xi3b, xi4b, xi5b, xi6b];
