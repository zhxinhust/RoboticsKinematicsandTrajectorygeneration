function J = jacobian(theta)
% «Û—≈øÀ±»æÿ’Û

global Tu;

% 
a2 = 400;
d1 = 830;
a3 = 1175;
a4 = 250;
d4 = 1125.33;
d6 = 230;

w1 = [0 0 -1];
w2 = [0 1 0];
w3 = [0 -1 0];
w4 = [-1 0 0];
w5 = [0 1 0];
w6 = [-1 0 0];

p1 = [0, 0, 0];
p2 = [a2, 0, d1];
p3 = [a2, 0, d1 + a3];
p4 = [0, 0, d1 + a3 + a4];
p5 = [a2 + d4, 0, d1 + a3 + a4];
p6 = [0, 0, d1 + a3 + a4];

		
A1 = POE(w1, p1, theta(1));
A2 = POE(w2, p2, theta(2));
A3 = POE(w3, p3, theta(3) + theta(2) + pi / 2);
A4 = POE(w4, p4, theta(4));
A5 = POE(w5, p5, theta(5));
A2b = POE(w3, p3, theta(2));
A3b = POE(w3, p3, theta(3) + pi / 2);
A2bb = A2 * A2b;

xi1 = [-cross(w1, p1), w1]';
xi2 = [-cross(w2, p2), w2]';
xi3 = [-cross(w3, p3), w3]';
xi4 = [-cross(w4, p4), w4]';
xi5 = [-cross(w5, p5), w5]';
xi6 = [-cross(w6, p6), w6]';
xi2b = [a3, 0, 0, 0, 0, 0]';


xi1b = Adjointmat(inv(Tu)) * xi1;
xi2b = Adjointmat(Tu \ A1) * (xi2 + xi3);
xi3b = Adjointmat(Tu \ A1 * A2bb) * xi3;
xi4b = Adjointmat(Tu \ A1 * A2 * A3) * xi4;
xi5b = Adjointmat(Tu \ A1 * A2 * A3 * A4) * xi5;
xi6b = Adjointmat(Tu \ A1 * A2 * A3 * A4 * A5) * xi6;

J = [xi1b, xi2b, xi3b, xi4b, xi5b, xi6b];
