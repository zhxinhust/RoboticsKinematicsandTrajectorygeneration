function R = forwardkinamicsPOE(theta)
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

gst0 = [1, 0, 0, a1 - a3;
		0, 1, 0, 0;
		0, 0, 1, d0 + a2 + d4 + d6;
		0, 0, 0, 1];
    
R = A1 * A2 * A3 * A4 * A5 * A6 * gst0;

end