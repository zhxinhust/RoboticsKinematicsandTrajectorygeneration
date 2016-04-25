function [x, y, z] = createcylinder(p1, p2, R)

[xc, yc, zc] = cylinder(R, 50);

p1d = p1(1:3);
p2d = p2(1:3);

d = norm(p1d - p2d);
zc = d * zc;

w = cross([0, 0, 1]', (p2d - p1d) / norm(p2d - p1d));
theta = acos(dot([0, 0, 1], (p2d -p1d) / norm(p2d - p1d)));

R = quat2dcm([cos(theta / 2), w' * sin(theta / 2)]);

cylinderp1(1, :) = xc(1, :);
cylinderp1(2, :) = yc(1, :);
cylinderp1(3, :) = zc(1, :);

cylinderp2(1, :) = xc(2, :);
cylinderp2(2, :) = yc(2, :);
cylinderp2(3, :) = zc(2, :);

cylinderp1rotated = R * cylinderp1;
cylinderp2rotated = R * cylinderp2;

x(1, :) = cylinderp1rotated(1, :) + p1(1);
x(2, :) = cylinderp2rotated(1, :) + p1(1);
y(1, :) = cylinderp1rotated(2, :) + p1(2);
y(2, :) = cylinderp2rotated(2, :) + p1(2);
z(1, :) = cylinderp1rotated(3, :) + p1(3);
z(2, :) = cylinderp2rotated(3, :) + p1(3);