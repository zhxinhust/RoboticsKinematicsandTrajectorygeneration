clear;
close all;

addpath('../Data/Input');
addpath('../Common');
addpath('../trajectorygeneration');

pathdata = importdata('toolpath.txt');
% pathdata = importdata('pathvol.txt');

pathdata = approximateanddensify(pathdata);

pathdata(:, 7:12) = pathdata(:, 7:12) * 20 / 10;    % 之前给定的速度值太小了，这里改大一点。


Ts = 0.002;

global Tu;
global Tt;

toolcoor = [352.64,0.00,469.28,0.00,60.00,180.00];  % 刀具坐标系数据
usercoor = [1613.56, 0, 780, 0, 0, 0];              % 用户坐标系数据

Tu = enlerangle2rotatemat(usercoor(1:3), usercoor(4:6));    % 求相应
Tt = enlerangle2rotatemat(toolcoor(1:3), toolcoor(4:6));

% 参考Liu H, Lai X, Wu W. Time-Optimal and Jerk-Continuous Trajectory
% Planning for Robot Manipulators with Kinematic Constraints. Robot Cim-Int
% Manuf, 2013, 2: 309-317中的数据
% vmax = [100, 95, 100, 150, 130, 110];
% amax = [45, 40, 75, 70, 90, 80];
% jmax = [60, 60, 55, 70, 75, 70];      % 论文中的跃度太小，导致采用双S型规划时陷入死循环

% vmax = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2];
% amax = [20, 20, 20, 20, 20, 20];
% jmax = [600, 600, 650, 600, 650, 600];

vmax = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2];
amax = [80, 80, 80, 80, 80, 80];
jmax = [1000, 1000, 1000, 1000, 1000, 1000];

lastjointpos = [0, 0, -pi / 2, 0, 0, 0];    % 保存上一关节空间为位置，在求逆解的时候选解用

maxindex = 1;
maxdis = 1000;


% 进行运动学反解，求关节位置和速度
% h = figure;
for k = 1:size(pathdata, 1)
    i = k;
    maxindex = 1;
    maxdis = 1000;
    
    Ttpos = enlerangle2rotatemat(pathdata(i, 1:3), pathdata(i, 4:6)); % 计算用户坐标系下的位姿
    Tbase = Tu * Ttpos / Tt;    % 变换到基座标系下的机器人末端处的描述，方便运动学逆解
    
    jp = inversekinamicsDH(Tbase);
    
    % 选解，使得与上一解距离最小
    for j = 1:size(jp, 1)
        temp = norm(jp(j, :) - lastjointpos);
        if temp < maxdis
            maxindex = j;
            maxdis = temp;
        end
    end
    joint(i, :) = jp(maxindex, :);
    
    Jaco = jacobian(joint(i, :));   % 求雅克比矩阵
    
    vjoint(i, :) = Jaco \ pathdata(i, 7:12)';   % 求各关节速度值
    
    lastjointpos = jp(maxindex, :);
end

aa = 1;

% 利用反解后关节空间中的位置进行轨迹规划
traj = jointtrajectorygeneration(joint, vjoint, vmax, amax, jmax);

% 插补计算轨迹
[jointq, jointv, jointa, jointj] = caltrajectorypoints(traj, joint, vjoint, Ts, jmax, pathdata);



% h = figure;
% k = 1;
% for i = 1:size(jointq, 1) / k
%     jointmove = jointq(i * k, :);
%     drawrobotpos(jointmove, h);
% end