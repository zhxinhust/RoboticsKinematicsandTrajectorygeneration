clear;
close all;

Ts = 0.002;

addpath('../Common/BSpline');
addpath('../Common/Robotics');
addpath('../Common/Dualquaternions');
addpath('../Common/Trajectorygeneration');
addpath('../Data/Input');
addpath('../Common\feedratescheduling/dynamicbasedjerklimited');
jnt = importdata('jnt2.txt');
pos = importdata('pos2.txt');
jac = importdata('jac2.txt');

jnt = jnt * pi / 180;
pos(:, 4:6) = pos(:, 4:6) * pi / 180;

span = 7;
jnt(:, 1) = smooth(jnt(:, 1), span);
jnt(:, 2) = smooth(jnt(:, 2), span);
jnt(:, 3) = smooth(jnt(:, 3), span);
jnt(:, 4) = smooth(jnt(:, 4), span);
jnt(:, 5) = smooth(jnt(:, 5), span);
jnt(:, 6) = smooth(jnt(:, 6), span);

pos(:, 1) = smooth(pos(:, 1), span);
pos(:, 2) = smooth(pos(:, 2), span);
pos(:, 3) = smooth(pos(:, 3), span);
pos(:, 4) = smooth(pos(:, 4), span);
pos(:, 5) = smooth(pos(:, 5), span);
pos(:, 6) = smooth(pos(:, 6), span);

vforward = zeros(size(jnt, 1) - 1, 3);
point = zeros(size(jnt, 1), 3);
angles = zeros(size(jnt, 1), 3);

vdiff = (pos(2 : end, :) - pos(1 : end - 1, :)) / Ts;

vdiff2 = vdiff;
for i = 1:size(jnt, 1)
    g = forwardkinamicsDH(jnt(i, :));
    point(i, :) = g(1:3, 4)';
    poserr(i) = norm(point(i, :) - pos(i, 1:3));
    aa = rotatemat2enlerangle(g);
    angles(i, :) = aa(1, :);
    
    if i > 1
        vdiffabs(i) = norm(pos(i, 1:3) - pos(i - 1, 1:3)) / Ts;
        vforward(i - 1, :) = (point(i, :) - point(i - 1, :)) / Ts;
        vforwardabs(i) = norm(vforward(i - 1, :));
    end
end

% vel = zeros(size(jnt, 1) - 1, 6);
velcart = zeros(size(jnt, 1) - 1, 6);
v = zeros(size(jnt, 1) - 1, 1);
vel = (jnt(2:end, :) - jnt(1:end - 1, :)) / Ts; 

% vel(:, 1) = smooth(vel(:, 1), span);
% vel(:, 2) = smooth(vel(:, 2), span);
% vel(:, 3) = smooth(vel(:, 3), span);
% vel(:, 4) = smooth(vel(:, 4), span);
% vel(:, 5) = smooth(vel(:, 5), span);
% vel(:, 6) = smooth(vel(:, 6), span);

jmas = zeros(6, 6);
for i = 1:size(jnt, 1) - 1
%     vel(i, :) = (jnt(i + 1, :) - jnt(i, :)) / Ts;
    
    jmas(1, :) = jac(i, 1:6);
    jmas(2, :) = jac(i, 7:12);
    jmas(3, :) = jac(i, 13:18);
    jmas(4, :) = jac(i, 19:24);
    jmas(5, :) = jac(i, 25:30);
    jmas(6, :) = jac(i, 31:36);
    
    Jac = jacobiananalytical(jnt(i, :), pos(i, 4), pos(i, 5));
%     Jac = jacobiangeometric(jnt(i, :));
%     jacobiangeometric2(jnt(i, :))
%     jacerr(:, :, i) = Jac - jmas;
    
    velcart(i, :) = (Jac * vel(i, :)')';
    v(i) = norm(velcart(i, 1:3));    
end

figure;

subplot(2, 2, 1);
plot(v, 'b');
title('笛卡尔空间速度大小');
hold on;
plot(vforwardabs, 'r');
plot(vdiffabs, 'g');

subplot(2, 2, 2);
plot(velcart(:, 1), 'b');
title('笛卡尔空间X方向速度');
hold on;
plot(vforward(:, 1), 'r');
plot(vdiff(:, 1), 'g');

subplot(2, 2, 3);
plot(velcart(:, 2), 'b');
title('笛卡尔空间Y方向速度');
hold on;
plot(vforward(:, 2), 'r');
plot(vdiff(:, 2), 'g');

subplot(2, 2, 4);
plot(velcart(:, 3), 'b');
title('笛卡尔空间Z方向速度');
hold on;
plot(vforward(:, 3), 'r');
plot(vdiff(:, 3), 'g');

figure;
subplot(2, 2, 1);
plot(vdiff2(:, 4), 'r');
hold on;
plot(velcart(:, 4), 'b');

subplot(2, 2, 2);
plot(vdiff2(:, 5), 'r');
hold on;
plot(velcart(:, 5), 'b');

subplot(2, 2, 3);
plot(vdiff2(:, 6), 'r');
hold on;
plot(velcart(:, 6), 'b');
acc = zeros(size(jnt, 1) - 2, 6);
for i = 1:size(jnt, 1) - 2
    acc(i, :) = (vel(i + 1, :) - vel(i, :)) / Ts;
end

span = 7;
acc(:, 1) = smooth(acc(:, 1), span);
acc(:, 2) = smooth(acc(:, 2), span);
acc(:, 3) = smooth(acc(:, 3), span);
acc(:, 4) = smooth(acc(:, 4), span);
acc(:, 5) = smooth(acc(:, 5), span);
acc(:, 6) = smooth(acc(:, 6), span);


figure;
plot3(point(:, 1), point(:, 2), point(:, 3));
title('笛卡尔空间轨迹');

figure;

subplot(2, 3, 1);
plot(jnt(:, 1));
title('关节空间轨迹1轴');

subplot(2, 3, 2);
plot(jnt(:, 2));
title('关节空间轨迹2轴');

subplot(2, 3, 3);
plot(jnt(:, 3));
title('关节空间轨迹3轴');

subplot(2, 3, 4);
plot(jnt(:, 4));
title('关节空间轨迹4轴');

subplot(2, 3, 5);
plot(jnt(:, 5));
title('关节空间轨迹5轴');

subplot(2, 3, 6);
plot(jnt(:, 6));
title('关节空间轨迹6轴');

figure;
subplot(2, 3, 1);
plot(vel(:, 1));
title('关节空间速度1轴');

subplot(2, 3, 2);
plot(vel(:, 2));
title('关节空间速度2轴');

subplot(2, 3, 3);
plot(vel(:, 3));
title('关节空间速度3轴');

subplot(2, 3, 4);
plot(vel(:, 4));
title('关节空间速度4轴');

subplot(2, 3, 5);
plot(vel(:, 5));
title('关节空间速度5轴');

subplot(2, 3, 6);
plot(vel(:, 6));
title('关节空间速度6轴');

figure;
subplot(2, 3, 1);
plot(acc(:, 1));
title('关节空间加速度1轴');


subplot(2, 3, 2);
plot(acc(:, 2));
title('关节空间加速度2轴');

subplot(2, 3, 3);
plot(acc(:, 3));
title('关节空间加速度3轴');

subplot(2, 3, 4);
plot(acc(:, 4));
title('关节空间加速度4轴');

subplot(2, 3, 5);
plot(acc(:, 5));
title('关节空间加速度5轴');

subplot(2, 3, 6);
plot(acc(:, 6));
title('关节空间加速度6轴');