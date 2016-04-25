function [der1, der2] = DerCalFromQ(p1, dualQder1, dualQder2, dualQ)
% 求二阶导矢

Q = dualQ(1:4);
R = dualQ(5:8);
Qs = Q;
Qs(2:4) = -Q(2:4);
Rs = R;
Rs(2:4) = -R(2:4);

Qder1 = dualQder1(1:4);
Rder1 = dualQder1(5:8);
Qder1s = Qder1;
Qder1s(2:4) = -Qder1(2:4);
Rder1s = Rder1;
Rder1s(2:4) = -Rder1(2:4);

Qder2 = dualQder2(1:4);
Rder2 = dualQder2(5:8);
Qder2s = Qder2;
Qder2s(2:4) = -Qder2(2:4);
Rder2s = Rder2;
Rder2s(2:4) = -Rder2(2:4);


% 将初始点表示为四元数格式
p = [0, p1];

f = quatmultiply(quatmultiply(Q, p), Qs) - quatmultiply(R, Qs) + quatmultiply(Q, Rs);

fd = quatmultiply(quatmultiply(Qder1, p), Qs) + quatmultiply(quatmultiply(Q, p), Qder1s) - quatmultiply(Rder1, Qs) - quatmultiply(R, Qder1s) + quatmultiply(Qder1, Rs) + quatmultiply(Q, Rder1s);

fdd = quatmultiply(quatmultiply(Qder2, p), Qs) + 2 * quatmultiply(quatmultiply(Qder1, p), Qder1s) + quatmultiply(quatmultiply(Q, p), Qder2s) - quatmultiply(Rder2, Qs) - 2 * quatmultiply(Rder1, Qder1s) - quatmultiply(R, Qder2s) + quatmultiply(Qder2, Rs) + 2 * quatmultiply(Qder1, Rder1s) + quatmultiply(Q, Rder2s);

g = quatmultiply(Q, Qs);

gd = quatmultiply(Qder1, Qs) + quatmultiply(Q, Qder1s);

gdd = quatmultiply(Qder2, Qs) + 2 * quatmultiply(Qder1, Qder1s) + quatmultiply(Q, Qder2s);

normQ = dot(Q, Q);

der1Temp = (quatmultiply(fd, g) - quatmultiply(f, gd)) / normQ^2;

der2Tmep = (quatmultiply(quatmultiply(fdd, g) - quatmultiply(f, gdd), g) - 2 * quatmultiply(quatmultiply(fd, g) - quatmultiply(f, gd), gd)) / normQ^3;

der1 = der1Temp(2:4);
der2 = der2Tmep(2:4);
