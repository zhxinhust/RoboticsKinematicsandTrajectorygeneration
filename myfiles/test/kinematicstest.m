clear;
close all;

addpath('../../Common');

theta = [-1.766, 3.209, -96.103, 25.093, 9.772, -16.229] * pi / 180;
AER = [-2.505, 104.931, -9.64] / 180 * pi;

% theta = [-1.409, 3.209, -93.374, 24.139, 10.636, -17.984] * pi / 180;
% AER = [-3.035, 103.060, -6.792] / 180 * pi;

% theta = [-0.75, 0, -90.003, 0.001, 13.398, 0] * pi / 180;
% AER = [0.749, 103.401, -0.001] / 180 * pi;

g = forwardkinamicsDH(theta);

R = g(1:3, 1:3);
p = g(1:3, 4)



AERm1 = [cos(AER(1)),-sin(AER(1)), 0;
         sin(AER(1)), cos(AER(1)), 0; 
         0,           0,           1];
     
AERm2 = [cos(AER(2)), 0, sin(AER(2)); 
         0,           1, 0; 
         -sin(AER(2)), 0, cos(AER(2))];
     
AERm3 = [cos(AER(3)), -sin(AER(3)), 0;
         sin(AER(3)), cos(AER(3)),  0; 
         0,           0,            1];
     
Rc5g = AERm1 * AERm2 * AERm3;
gc5g = zeros(4, 4);
gc5g(1:3, 1:3) = Rc5g;
gc5g(1:3, 4) = p;
gc5g(4, 4) = 1;

[enlerangle, p] = rotatemat2enlerangle(g)
% [r1, r2, r3] = dcm2angle(R, 'ZYZ');
% r1 = r1 * 180 / pi
% r2 = r2 * 180 / pi
% r3 = r3 * 180 / pi
