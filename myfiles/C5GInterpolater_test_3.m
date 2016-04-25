clear;
close all;

Ts = 0.002;

addpath('../Common/BSpline');
addpath('../Common/Robotics');
addpath('../Common/Dualquaternions');
addpath('../Common/Trajectorygeneration');
addpath('../Data/Input');
addpath('../Common\feedratescheduling/dynamicbasedjerklimited');
jnt = importdata('jnt3.txt');
pos = importdata('pos3.txt');
jac = importdata('jac3.txt');

jnt = jnt * pi / 180;
pos(:, 4:6) = pos(:, 4:6) * pi / 180;

span = 5;  % 求平均的点数

for i = 1:6
    % 平均去噪
    jnt(:, i) = smooth(jnt(:, i), span);
    pos(:, i) = smooth(pos(:, i), span);
end

vforward = zeros(size(jnt, 1) - 1, 3);
point = zeros(size(jnt, 1), 3);
angles = zeros(size(jnt, 1), 3);

vdiff = (pos(2 : end, :) - pos(1 : end - 1, :)) / Ts;   % 笛卡尔坐标差分

vdiff2 = vdiff;
vdiffabs = zeros(size(jnt, 1) - 1, 1);
vforwardabs = zeros(size(jnt, 1) - 1, 1);

for i = 1:size(jnt, 1)
    g = forwardkinamicsDH(jnt(i, :));
    point(i, :) = g(1:3, 4)';
%    poserr(i) = norm(point(i, :) - pos(i, 1:3));
    aa = rotatemat2enlerangle(g);
    angles(i, :) = aa(1, :);
    
    if i > 1
        vdiffabs(i) = norm(pos(i, 1:3) - pos(i - 1, 1:3)) / Ts;
        vforward(i - 1, :) = (point(i, :) - point(i - 1, :)) / Ts;
        vforwardabs(i) = norm(vforward(i - 1, :));
        Temp = [0, -sin(pos(i - 1, 4)), cos(pos(i - 1, 4)) * sin(pos(i - 1, 5));
                0, cos(pos(i - 1, 4)), sin(pos(i - 1, 4)) * sin(pos(i - 1, 5));
                1, 0, cos(pos(i - 1, 5))];
        vdiff(i - 1, 4:6) = (Temp * vdiff(i - 1, 4:6)')'; % 将分析速度转化为标准的角速度
    end
end

velcart = zeros(size(jnt, 1) - 1, 6);
v = zeros(size(jnt, 1) - 1, 1);
vel = (jnt(2:end, :) - jnt(1:end - 1, :)) / Ts;     % 差分得到关节空间速度
for i = 1:6
    % 均值平滑
    vel(:, i) = smooth(vel(:, i), span);
    vdiff(:, i) = smooth(vdiff(:, i), span);
end

acc = (vel(2:end, :) - vel(1:end - 1, :)) / Ts;     % 差分得到关节加速度 
accdiff = (vdiff(2:end, :) - vdiff(1:end - 1, :)) / Ts;
for i = 1:6
    acc(:, i) = smooth(acc(:, i), span);
    accdiff(:, i) = smooth(accdiff(:, i), span);
end

jerkdiff = (accdiff(2:end, :) - accdiff(1:end - 1, :)) / Ts;
jerk = (acc(2:end, :) - acc(1:end - 1, :)) / Ts;
for i = 1:6
    jerkdiff(:, i) = smooth(jerkdiff(:, i), span);
    jerk(:, i) = smooth(jerk(:, i), span);
end

jmas = zeros(6, 6);
Jder = zeros(6, 5);
accjder = zeros(size(acc, 1), 6);
a = zeros(size(acc, 1), 1);
for i = 1:size(jnt, 1) - 1
    jmas(1, :) = jac(i, 1:6);
    jmas(2, :) = jac(i, 7:12);
    jmas(3, :) = jac(i, 13:18);
    jmas(4, :) = jac(i, 19:24);
    jmas(5, :) = jac(i, 25:30);
    jmas(6, :) = jac(i, 31:36);
    
    Jac = jacobiangeometric(jnt(i, :));
    
%     Jder = zeros(6, 6);
%     for j = 1:6
%         Jder = Jder + vel(i, j) * Jdifferentiation_j(jnt(i, :), j);
%     end
    
    velcart(i, :) = (Jac * vel(i, :)')';
%     if i < size(jnt, 1) - 1
%         accjder(i, :) = (Jder * vel(i, :)' + Jac * acc(i, :)')';
%     end
    
    v(i) = norm(velcart(i, 1:3));    
end

acccart = (velcart(2:end, :) - velcart(1:end - 1, :))/Ts;
for i = 1:6
    acccart(:, i) = smooth(acccart(:, i), span);
    accjder(:, i) = smooth(accjder(:, i), span);
end

for i = 1:size(acccart, 1)
    a(i) = norm(acccart(i, 1:3));
end

jettison = 150;

% 绘制笛卡尔空间各个方向加速度分量
figure;
subplot(2, 2, 1);
plot(a);

subplot(2, 2, 2);
plot(acccart(jettison:end - jettison, 1), 'b');
hold on;
plot(accdiff(jettison:end - jettison, 1), 'r');
plot(accjder(jettison:end - jettison, 1), 'g');
title('X方向加速度大小');

subplot(2, 2, 3);
plot(acccart(jettison:end - jettison, 2), 'b');
hold on;
plot(accdiff(jettison:end - jettison, 2), 'r');
plot(accjder(jettison:end - jettison, 2), 'g');
title('Y方向加速度大小');

subplot(2, 2, 4);
plot(acccart(jettison:end - jettison, 3), 'b');
hold on;
plot(accdiff(jettison:end - jettison, 3), 'r');
plot(accjder(jettison:end - jettison, 3), 'g');
title('Z方向加速度大小');

% figure;
% subplot(2, 2, 1);
% plot(acccart(jettison:end - jettison, 4), 'b');
% hold on;
% plot(accdiff(jettison:end - jettison, 4), 'r');
% plot(accjder(jettison:end - jettison, 4), 'g');
% 
% subplot(2, 2, 2);
% plot(acccart(jettison:end - jettison, 5), 'b');
% hold on;
% plot(accdiff(jettison:end - jettison, 5), 'r');
% plot(accjder(jettison:end - jettison, 5), 'g');
% 
% subplot(2, 2, 3);
% plot(acccart(jettison:end - jettison, 6), 'b');
% hold on;
% plot(accdiff(jettison:end - jettison, 6), 'r');
% plot(accjder(jettison:end - jettison, 6), 'g');
% 
% % 绘制笛卡尔空间各跃度分量
% figure;
% subplot(2, 2, 1);
% plot(jerkdiff(jettison:end - jettison, 1));
% 
% subplot(2, 2, 2);
% plot(jerkdiff(jettison:end - jettison, 2));
% 
% subplot(2, 2, 3);
% plot(jerkdiff(jettison:end - jettison, 3));

% 绘制各轴加速度情况
figure;
subplot(2, 3, 1);
plot(acc(jettison:end - jettison, 1));
subplot(2, 3, 2);
plot(acc(jettison:end - jettison, 2));
subplot(2, 3, 3);
plot(acc(jettison:end - jettison, 3));
subplot(2, 3, 4);
plot(acc(jettison:end - jettison, 4));
subplot(2, 3, 5);
plot(acc(jettison:end - jettison, 5));
subplot(2, 3, 6);
plot(acc(jettison:end - jettison, 6));

% 绘制各轴加速度情况
figure;
subplot(2, 3, 1);
plot(vel(jettison:end - jettison, 1));
subplot(2, 3, 2);
plot(vel(jettison:end - jettison, 2));
subplot(2, 3, 3);
plot(vel(jettison:end - jettison, 3));
subplot(2, 3, 4);
plot(vel(jettison:end - jettison, 4));
subplot(2, 3, 5);
plot(vel(jettison:end - jettison, 5));
subplot(2, 3, 6);
plot(vel(jettison:end - jettison, 6));


figure;
subplot(2, 3, 1);
plot(jerk(jettison:end - jettison, 1));
subplot(2, 3, 2);
plot(jerk(jettison:end - jettison, 2));
subplot(2, 3, 3);
plot(jerk(jettison:end - jettison, 3));
subplot(2, 3, 4);
plot(jerk(jettison:end - jettison, 4));
subplot(2, 3, 5);
plot(jerk(jettison:end - jettison, 5));
subplot(2, 3, 6);
plot(jerk(jettison:end - jettison, 6));