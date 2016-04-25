function [enlerangle, p] = rotatemat2enlerangle(g)

p = g(1:3, 4)';

r1 = zeros(2, 1);
r2 = zeros(2, 1);
r3 = zeros(2, 1);

r1(1) = atan2(g(2, 3), g(1, 3));

if r1(1) > 0
    r1(2) = r1(1) - pi;
else
    r1(2) = r1(1) + pi;
end

r2(1) = atan2(g(1, 3) * cos(r1(1)) + g(2, 3) * sin(r1(1)), g(3, 3));
r2(2) = atan2(g(1, 3) * cos(r1(2)) + g(2, 3) * sin(r1(2)), g(3, 3));

if r2(1) < 0
    r2(1) = r2(1) + 2 * pi;
end

if r2(2) < 0
    r2(2) = r2(2) + 2 * pi;
end

r3(1) = atan2(-g(1, 1) * sin(r1(1)) + g(2, 1) * cos(r1(1)), -g(1, 2) * sin(r1(1)) + g(2, 2) * cos(r1(1)));
r3(2) = atan2(-g(1, 1) * sin(r1(2)) + g(2, 1) * cos(r1(2)), -g(1, 2) * sin(r1(2)) + g(2, 2) * cos(r1(2)));



enlerangle = [r1, r2, r3] * 180 / pi;