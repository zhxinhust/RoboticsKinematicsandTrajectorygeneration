function m = POE(w, p, theta)
% 根据转轴向量w, v = -w x q和theta求其对应的其次变换矩阵

% 将w, v 化为列向量
% wc = zeros(3, 1);
% vc = zeros(3, 1);
% 
% for i = 1:3
%     wc(i) = w(i);
%     vc(i) = v(i);
% end
v = -cross(w, p);
wc = w.';
vc = v.';

R = eye(3) + hatm(w) * sin(theta) + hatm(w) * hatm(w) * (1 - cos(theta));

q = (eye(3) - R) * cross(wc, vc) +  wc * dot(w, v) * theta;


m(1:3, 1:3) = R;
m(1:3, 4) = q;
m(4, 4) = 1;