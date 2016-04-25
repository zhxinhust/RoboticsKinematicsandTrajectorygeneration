function Ad = Adjointmat(g)
% 根据变换矩阵g求其伴随变换矩阵

R = g(1:3, 1:3);
p = g(1:4, 4)';

phat = hatm(p);

Ad = zeros(6, 6);

Ad(1:3, 1:3) = R;
Ad(4:6, 4:6) = R;
Ad(1:3, 4:6) = phat * R;