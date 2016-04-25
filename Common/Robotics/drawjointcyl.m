function drawjointcyl(xc, yc, zc, g, facecolor, pathcolor)
% 绘制代表关节的圆柱，xc, yc, zc为中心点在原点处的圆柱
% g为变换矩阵，facecolor为柱面颜色，pathcolor为两端面颜色

cylinderp1 = ones(4, size(xc, 2));
cylinderp2 = ones(4, size(xc, 2));

cylinderp1(1, :) = xc(1, :);
cylinderp1(2, :) = yc(1, :);
cylinderp1(3, :) = zc(1, :);

cylinderp2(1, :) = xc(2, :);
cylinderp2(2, :) = yc(2, :);
cylinderp2(3, :) = zc(2, :);

% 坐标变换
cylinderp1rotated = g * cylinderp1;
cylinderp2rotated = g * cylinderp2;

x(1, :) = cylinderp1rotated(1, :);
x(2, :) = cylinderp2rotated(1, :);
y(1, :) = cylinderp1rotated(2, :);
y(2, :) = cylinderp2rotated(2, :);
z(1, :) = cylinderp1rotated(3, :);
z(2, :) = cylinderp2rotated(3, :);

surf(x, y, z, 'facecolor', facecolor, 'EdgeColor', 'none');  % 绘制柱面
patch(x', y', z', pathcolor, 'EdgeColor', 'none');     % 绘制端面
