clear;
addpath('.\Common');


data = importdata('results3.txt');
data = data * pi / 180;
for i = 1:size(data, 1)
    g = forwardkinamicsDH(data(i, :));
    point(i, :) = g(1:3, 4)';
end

for i = 1:size(data, 1) - 1
    vel(i, :) = data(i + 1, :) - data(i, :);
end
vel = vel / 0.0004;

span = 2;
vel(:, 1) = smooth(vel(:, 1), span);
vel(:, 1) = smooth(vel(:, 2), span);
vel(:, 1) = smooth(vel(:, 3), span);
vel(:, 1) = smooth(vel(:, 4), span);
vel(:, 1) = smooth(vel(:, 5), span);
vel(:, 1) = smooth(vel(:, 6), span);

for i = 1:size(data, 1) - 2
    acc(i, :) = vel(i + 1, :) - vel(i, :);
end

figure;
plot3(point(:, 1), point(:, 2), point(:, 3));

figure;
subplot(2, 3, 1);
plot(data(:, 1));

subplot(2, 3, 2);
plot(data(:, 2));

subplot(2, 3, 3);
plot(data(:, 3));

subplot(2, 3, 4);
plot(data(:, 4));

subplot(2, 3, 5);
plot(data(:, 5));

subplot(2, 3, 6);
plot(data(:, 6));

figure;
subplot(2, 3, 1);
plot(vel(:, 1));

subplot(2, 3, 2);
plot(vel(:, 2));

subplot(2, 3, 3);
plot(vel(:, 3));

subplot(2, 3, 4);
plot(vel(:, 4));

subplot(2, 3, 5);
plot(vel(:, 5));

subplot(2, 3, 6);
plot(vel(:, 6));

figure;
subplot(2, 3, 1);
plot(acc(:, 1));

subplot(2, 3, 2);
plot(acc(:, 2));

subplot(2, 3, 3);
plot(acc(:, 3));

subplot(2, 3, 4);
plot(acc(:, 4));

subplot(2, 3, 5);
plot(acc(:, 5));

subplot(2, 3, 6);
plot(acc(:, 6));