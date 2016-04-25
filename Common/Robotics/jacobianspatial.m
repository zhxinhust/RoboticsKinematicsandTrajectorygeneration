function J = jacobianspatial(theta)
% «Û—≈øÀ±»æÿ’Û

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
A2 = POE(w2, p2, theta(2)) * POE(w3, p3, theta(2));
A3 = POE(w3, p3, theta(3));
A4 = POE(w4, p4, theta(4));
A5 = POE(w5, p5, theta(5));
A6 = POE(w6, p6, theta(6));

xi1 = [-cross(w1, p1), w1]';
xi2 = [-cross(w2, p2), w2]' + [-cross(w3, p3), w3]';
xi3 = [-cross(w3, p3), w3]';
xi4 = [-cross(w4, p4), w4]';
xi5 = [-cross(w5, p5), w5]';
xi6 = [-cross(w6, p6), w6]';

xi1b = xi1;
xi2b = Adjointmat(A1) * xi2;
xi3b = Adjointmat(A1 * A2) * xi3;
xi4b = Adjointmat(A1 * A2 * A3) * xi4;
xi5b = Adjointmat(A1 * A2 * A3 * A4) * xi5;
xi6b = Adjointmat(A1 * A2 * A3 * A4 * A5) * xi6;

J = [xi1b, xi2b, xi3b, xi4b, xi5b, xi6b];
