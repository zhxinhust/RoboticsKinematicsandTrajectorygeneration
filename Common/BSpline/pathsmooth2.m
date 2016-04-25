function [controlp, knotvector] = pathsmooth2(data, splineorder, controlpnum)
% 将离散刀路插值为样条刀路

Q = data;

m = size(data, 1) - 1;    % 要拟合的离散数据点最大下标
n = controlpnum - 1;        % 控制点数量最大下标

% 进行参数化
dSum = 0;
Ud = zeros(1, m);
Ud(1) = 0;
for i = 1:m
    Ud(i + 1) = sqrt(norm(Q(i, 1:3) - Q(i + 1, 1:3)));
    dSum = dSum + Ud(i + 1);
end

Ub = Ud / dSum;
for i = 1:m + 1
    if i > 1
        Ub(i) = Ub(i) + Ub(i - 1);
    end
end

U = zeros(1, n + splineorder + 2);

U(n + 1 : end) = 1;

d = (m + 1) / (n - splineorder + 1);
for j = 1 : n - splineorder
	i = round(j * d - 0.5);
	a = j * d - i;
	U(splineorder + j + 1) = (1 - a) * Ub(i) + a * Ub(i + 1);
end

NFul = zeros(m + 1, n + 1);
for i = 1 : m + 1
	for j = 1 : n + 1
		NFul(i, j) = GetBaseFunVal(Ub(i), j - 1, splineorder, U);
	end
end

N = NFul(2:m, 2:n);

Ri = Q(2 : m, :);
for i = 1 : m - 1
	N0pUbi = NFul(i + 1, 1);
    NnpUbi = NFul(i + 1, end);
    Ri(i, :) = Ri(i, :) - data(1, :) * N0pUbi - data(end, :) * NnpUbi;
end

CQ = (N' * N) \ (N' * Ri);

controlp = zeros(controlpnum, size(data, 2));
controlp(1, :) = data(1, :);
controlp(2 : end - 1, :) = CQ;
controlp(end, :) = data(end, :);

knotvector = U;