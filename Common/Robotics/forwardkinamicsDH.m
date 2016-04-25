function R = forwardkinamicsDH(theta)
% 

% 机器人各连杆参数值
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

% Ae = [-1, 0, 0, 0;
%        0, 1, 0, 0;
%        0, 0, -1, 0;
%        0, 0, 0, 1];

Ab = DHmatrix(0, d0, 0, alpha0); 
A1 = DHmatrix(theta(1), 0, a1, alpha1);
A2 = DHmatrix(theta(2) - pi / 2, 0, a2, alpha2);
A3 = DHmatrix(theta(3) + theta(2) + pi / 2, 0, a3, alpha3);
A4 = DHmatrix(theta(4), -d4, 0, alpha4);
A5 = DHmatrix(theta(5), 0, 0, alpha5);
A6 = DHmatrix(theta(6) + pi, -d6, 0, alpha6);

R = Ab * A1 * A2 * A3 * A4 * A5 * A6;

