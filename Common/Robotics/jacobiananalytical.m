function J = jacobiananalytical(theta, A, E)
% 求几何雅克比矩阵，没有工具坐标系

% 
a1 = 400;
d0 = 830;
a2 = 1175;
a3 = 250;
d4 = 1125.33;
d6 = 230;

w1 = [0 0 -1];
w2 = [0 1 0];
w3 = [0 -1 0];
w4 = [0 0 -1];
w5 = [0 1 0];
w6 = [0 0 -1];

p1 = [0, 0, 0];
p2 = [a1, 0, d0];
p3 = [a1, 0, d0 + a2];
p4 = [a1 - a3, 0, d0 + a2];
p5 = [a1 - a3, 0, d0 + a2 + d4];
p6 = [a1 - a3, 0, d0 + a2 + d4 + d6];
		
A1 = POE(w1, p1, theta(1));
A2 = POE(w2, p2, theta(2));
A3 = POE(w3, p3, theta(3) + theta(2));
A4 = POE(w4, p4, theta(4));
A5 = POE(w5, p5, theta(5));
A6 = POE(w6, p6, theta(6));

gst0 = [1, 0, 0, a1 - a3;
		0, 1, 0, 0;
		0, 0, 1, d0 + a2 + d4 + d6;
		0, 0, 0, 1];
    
p1b = [p1, 1]';
p2b = A1 * [p2, 1]';
p3b = A1 * A2 * [p3, 1]';
p4b = A1 * A2 * A3 * [p4, 1]';
p5b = A1 * A2 * A3 * A4 * [p5, 1]';
p6b = A1 * A2 * A3 * A4 * A5 * [p6, 1]';
pe  = A1 * A2 * A3 * A4 * A5 * A6 * gst0 * [0, 0, 0, 1]';

w1b = [w1, 0]';
w2b = [0, 0, 0, 0]';
w3b = A1 * [w3, 0]';
w4b = A1 * A2 * A3 * [w4, 0]';
w5b = A1 * A2 * A3 * A4 * [w5, 0]';
w6b = A1 * A2 * A3 * A4 * A5 * [w6, 0]';

xi2b = A1 * A2 * [a2, 0, 0, 0]';

xi1b = [cross(w1b(1:3), pe(1 : 3) - p1b(1 : 3)); w1b(1:3)];
xi2b = [xi2b(1:3); w2b(1:3)];
xi3b = [cross(w3b(1:3), pe(1 : 3) - p3b(1 : 3)); w3b(1:3)];
xi4b = [cross(w4b(1:3), pe(1 : 3) - p4b(1 : 3)); w4b(1:3)];
xi5b = [cross(w5b(1:3), pe(1 : 3) - p5b(1 : 3)); w5b(1:3)];
xi6b = [cross(w6b(1:3), pe(1 : 3) - p6b(1 : 3)); w6b(1:3)];

T = eye(6);
T(4:6, 4:6) = [0, -sin(A), cos(A) * sin(E);
               0, cos(A), sin(A) * sin(E);
               1, 0, cos(E)];

J = [xi1b, xi2b, xi3b, xi4b, xi5b, xi6b];

J = T \ J;
