function drawrobotpos(theta, h)
% 根据给出的关节角度绘制机器人
% theta为关节角度值，h为要绘制图形的figure句柄。可以用 h = figure 生成


global Tu;
global Tt;

% 采用DH模型
a1 = 400;
d0 = 830;
a2 = 1175;
a3 = 250;
d4 = -1125.33;
d6 = -230;

alpha0 = 180 / 180 * pi;
alpha1 = 90 / 180 * pi;
alpha2 = 180 / 180 * pi;
alpha3 = -90 / 180 * pi;
alpha4 = -90 / 180 * pi;
alpha5 = 90 / 180 * pi;
alpha6 = 180 / 180 * pi;


% 各关节的变换矩阵
Ab = DHmatrix(0, d0, 0, alpha0); 
A1 = DHmatrix(theta(1), 0, a1, alpha1);
A2 = DHmatrix(theta(2) - pi / 2, 0, a2, alpha2);
A3 = DHmatrix(theta(3) + theta(2) + pi/ 2, 0, a3, alpha3);
A4 = DHmatrix(theta(4), d4, 0, alpha4);
A5 = DHmatrix(theta(5), 0, 0, alpha5);
A6 = DHmatrix(theta(6) + pi, d6, 0, alpha6);

% 为统一，都采用其次表示
p0 = [0, 0, 0]';
p1 = [0, 0, d0]';
p2 = Ab * A1 * [0, 0, 0, 1]';
p3 = Ab * A1 * A2 * [0, 0, 0, 1]';
p4 = Ab * A1 * A2 * A3 * A4 * [0, 0, 0, 1]';
p5 = Ab * A1 * A2 * A3 * A4 * A5 * [0, 0, 0, 1]';
p6 = Ab * A1 * A2 * A3 * A4 * A5 * A6 * [0, 0, 0, 1]';
pt = Ab * A1 * A2 * A3 * A4 * A5 * A6 * Tt * [0, 0, 0, 1]';

p41 = Ab * A1 * A2 * A3 * A4 * [0, d4, 0, 1]';

figure(h);



cla(gca);   % 清除原figure上的图形

% t1 = tic;

linewidth = 3;  % 绘制连杆的线宽

% 绘制连杆
plot3([p0(1), p1(1)], [p0(2), p1(2)], [p0(3), p1(3)], 'b', 'linewidth', linewidth);
hold on;
plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'k', 'linewidth', linewidth);

plot3([p2(1), p3(1)], [p2(2), p3(2)], [p2(3), p3(3)], 'k', 'linewidth', linewidth);

plot3([p2(1), p3(1)], [p2(2), p3(2)], [p2(3), p3(3)], 'k', 'linewidth', linewidth);

plot3([p3(1), p41(1)], [p3(2), p41(2)], [p3(3), p41(3)], 'k', 'linewidth', linewidth);

plot3([p41(1), p4(1)], [p41(2), p4(2)], [p41(3), p4(3)], 'k', 'linewidth', linewidth);

plot3([p4(1), p5(1)], [p4(2), p5(2)], [p4(3), p5(3)], 'k', 'linewidth', linewidth);

plot3([p5(1), p6(1)], [p5(2), p6(2)], [p5(3), p6(3)], 'k', 'linewidth', linewidth);

plot3([pt(1), p6(1)], [pt(2), p6(2)], [pt(3), p6(3)], 'color', [0.5 0.5 0.5], 'linewidth', linewidth);

% 绘制关节
h = 250;    % 关节圆柱的长度
R = 70;     % 关节圆柱的半径

[xc, yc, zc] = cylinder(R, 20);
zc = zc * h - h / 2;    % 将圆柱平移到原点为中心高度处

facecolor = [0.7, 0, 0];    % 柱面颜色
pathcolor = [0.4, 0, 0];    % 柱端面颜色

% 绘制第一关节
drawjointcyl(xc, yc, zc, Ab, facecolor, pathcolor);

% 绘制第二关节
drawjointcyl(xc, yc, zc, Ab * A1, facecolor, pathcolor);

% 绘制第三关节
drawjointcyl(xc, yc, zc, Ab * A1 * A2, facecolor, pathcolor);

% 绘制第四关节
drawjointcyl(xc, yc, zc, Ab * A1 * A2 * A3, facecolor, pathcolor);

% 绘制第五关节
drawjointcyl(xc, yc, zc, Ab * A1 * A2 * A3 * A4, facecolor, pathcolor);

% 绘制第六关节
drawjointcyl(xc, yc, zc, Ab * A1 * A2 * A3 * A4 * A5, facecolor, pathcolor);

% 绘制坐标系
len = 500;  % 代表坐标系的线段长度
axiswidth = 1.5;    % 坐标系线宽
axisfontsize = 12;  % 字号
% 绘制基座标系
plot3([0, len], [0, 0], [0, 0], 'r', 'linewidth', axiswidth);
text(len, 0, 0, 'Xb', 'fontsize', axisfontsize);

plot3([0, 0], [0, len], [0, 0], 'b', 'linewidth', axiswidth);
text(0, len, 0, 'Yb', 'fontsize', axisfontsize);

plot3([0, 0], [0, 0], [0, len], 'g', 'linewidth', axiswidth);
text(0, 0, len, 'Zb', 'fontsize', axisfontsize);


len = 200;  % 代表坐标系的线段长度
% 绘制末端工具坐标系
ptx = [len, 0, 0, 1]';
pty = [0, len, 0, 1]';
ptz = [0, 0, len, 1]';

ptxb = Ab * A1 * A2 * A3 * A4 * A5 * A6 * Tt * ptx;
ptyb = Ab * A1 * A2 * A3 * A4 * A5 * A6 * Tt * pty;
ptzb = Ab * A1 * A2 * A3 * A4 * A5 * A6 * Tt * ptz;


plot3([pt(1), ptxb(1)], [pt(2), ptxb(2)], [pt(3), ptxb(3)], 'r', 'linewidth', axiswidth);
text(ptxb(1), ptxb(2), ptxb(3), 'Xt', 'fontsize', axisfontsize);

plot3([pt(1), ptyb(1)], [pt(2), ptyb(2)], [pt(3), ptyb(3)], 'b', 'linewidth', axiswidth);
text(ptyb(1), ptyb(2), ptyb(3), 'Yt', 'fontsize', axisfontsize);

plot3([pt(1), ptzb(1)], [pt(2), ptzb(2)], [pt(3), ptzb(3)], 'g', 'linewidth', axiswidth);
text(ptzb(1), ptzb(2), ptzb(3), 'Zt', 'fontsize', axisfontsize);

len = 500;  % 代表坐标系的线段长度
% 绘制末端工具坐标系
ptx = [len, 0, 0, 1]';
pty = [0, len, 0, 1]';
ptz = [0, 0, len, 1]';


puxb = Tu * ptx;
puyb = Tu * pty;
puzb = Tu * ptz;

puo = Tu * [0, 0, 0, 1]';

plot3([puo(1), puxb(1)], [puo(2), puxb(2)], [puo(3), puxb(3)], 'r', 'linewidth', axiswidth);
text(ptxb(1), ptxb(2), ptxb(3), 'Xt', 'fontsize', axisfontsize);

plot3([puo(1), puyb(1)], [puo(2), puyb(2)], [puo(3), puyb(3)], 'b', 'linewidth', axiswidth);
text(ptyb(1), ptyb(2), ptyb(3), 'Yt', 'fontsize', axisfontsize);

plot3([puo(1), puzb(1)], [puo(2), puzb(2)], [puo(3), puzb(3)], 'g', 'linewidth', axiswidth);
text(ptzb(1), ptzb(2), ptzb(3), 'Zt', 'fontsize', axisfontsize);

% toc(t1)
% 设置绘图区域
xlim([-(a1 - d4 - d6) - 300  a1 - d4 - d6 + 300]);
ylim([-(a1 - d4 - d6) - 300  a1 - d4 - d6 + 300]);
zlim([0  d0 + a2 - a3 - d4 - d6 + 300]);

grid on;

xlabel('X');
ylabel('Y');
zlabel('Z');
