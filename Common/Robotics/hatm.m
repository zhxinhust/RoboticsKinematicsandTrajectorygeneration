function m = hatm(a)
% 叉积线性算子，即将三维向量取hat

m = [0, -a(3), a(2); 
    a(3), 0, -a(1);
    -a(2), a(1), 0];

