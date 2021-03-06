function feedratemax = NurbsScanningRobot(Bsplinepath, interpolationperiod)

global Tu;  % 用户坐标系
global Tt;  % 工具坐标系
global jointinit;   % 初始点关节值

global arcLength;

global simpsonVector;       % 用来保存用辛普森公式迭代计算出来的(u, l)值
global simpsonVectorIndex;  % 此变量为上数据点的数量

arcLength = 0;
simpsonVector = 0;
simpsonVectorIndex = 1;

global knotvector;
global controlp;
global splineorder;

controlp = Bsplinepath.controlp;
knotvector = Bsplinepath.knotvector;
splineorder = Bsplinepath.splineorder;

% 设定约束条件
chorderror = 0.0002;
% F = 100;

axismaxvel = [0.3491, 0.3142, 0.3840, 0.4538, 0.4538, 0.6807] * 0.8;
axismaxacc = [0.2648, 0.2793, 0.2639, 0.1606, 0.1518, 0.2269] * 0.8;
axismaxjerk = [0.6468, 0.7645, 0.8739, 0.2094, 0.1693, 0.2269] * 0.8;

maxfeedrate = 100;

% knotvector = BSplinepath.knotvector;
% controlp = BSplinepath.controlp;
% splineorder = BSplinepath.splineorder;

Ts = interpolationperiod;   % 插补周期

stepnum = 1;    % 记录点数

u = 0; du = 0.0005;
uarr = 0:du:1;
Jarr = zeros(6, 6, length(uarr));   % 保存分析雅克比矩阵，减少重复计算
TderA = zeros(6, 6, length(uarr));
TderE = zeros(6, 6, length(uarr));
T = zeros(6, 6, length(uarr));
Jder = zeros(6, 6, 6, length(uarr));

feedratemax = zeros(length(uarr));
error = zeros(length(uarr));

scanpders = zeros(length(uarr), 18);
joint = zeros(length(uarr), 6);

seglenth = zeros(length(uarr) - 1, 1);
               
for u = 0:du:1
   deboorp = DeBoorCoxNurbsCal(u, Bsplinepath, 2);
   % 保存数据
   scanpders(stepnum, 1:6) = deboorp(1, :);
   scanpders(stepnum, 7:12) = deboorp(2, :);
   scanpders(stepnum, 13:18) = deboorp(3, :);
   
   % 由于刀路文件描述的刀具坐标系在用户坐标系下的姿态，运动学逆解时需要转换为机器人末端在基座标系中的位姿
    g = Tu * enlerangle2rotatemat(deboorp(1, 1:3), deboorp(1, 4:6)) / Tt;    
    
    theta = inversekinamicsDH2( g); % 运动学逆解
    
    % 选解，这里采用相对于上一位置关节变化最小的点作为接
    mindis = 100;
    minindex = 1;
    if u == 0
%         minindex = 64;
        for j = 1:size(theta, 1)
            % 第一个点选与设定的初始位置最近的点
            if norm(theta(j, :) - jointinit) < mindis
                minindex = j;
                mindis = norm(theta(j, :) - joint(stepnum - 1, :));
            end
        end
    else
        for j = 1:size(theta, 1)
            % 第二个点开始选距离上一位置最近的点
            if norm(theta(j, :) - joint(stepnum - 1, :)) < mindis
                minindex = j;
                mindis = norm(theta(j, :) - joint(stepnum - 1, :));
            end
        end
    end
    joint(stepnum, :) = theta(minindex, :); % 计算当前位置处关节角度值
    
    % 求几何雅克比矩阵
    Jarr(:, :, stepnum) = jacobiangeometric(joint(stepnum, :));  
    p = scanpders(stepnum, 1:6);
    % 计算矩阵T
    T(1:3, 1:3, stepnum) = eye(3);
    T(4:6, 4:6, stepnum) = [0, -sin(p(4)), cos(p(4)) * sin(p(5));
               0, cos(p(4)), sin(p(4)) * sin(p(5));
               1, 0, cos(p(5))];
    
    % 计算dT/dA
    TderA(4, 5, stepnum) = -cos(p(4));
    TderA(4, 6, stepnum) = -sin(p(4)) * sin(p(5));
    TderA(5, 5, stepnum) = -sin(p(4));
    TderA(5, 6, stepnum) = cos(p(4)) * sin(p(5));
    
    % 计算dT/dE
    TderE(4, 6, stepnum) = -cos(p(4)) * cos(p(5));
    TderE(5, 6, stepnum) = sin(p(4)) * cos(p(5));
    TderE(6, 6, stepnum) = -sin(p(5));
    
    % 求dJ/d theta j
    for j = 1:6
        Jder(:, :, j, stepnum) = Jdifferentiation_j(joint(stepnum, :), j);
    end
    
    arcLength = 0;
    % 计算各段弧长    
    if u > 0
        IterativeCalArcLength2(u - du, u);
        seglenth(stepnum - 1) = arcLength;
    end
    
    pathderunit = deboorp(2, :) / norm(deboorp(2, 1:3));    % 得到单位化的速度量
    pathderunit(4:6) = pathderunit(4:6) / 180 * pi;
        
    temp = Jarr(:, :, stepnum) \ pathderunit';
    v = min(abs(axismaxvel) ./ abs(temp'));  % 计算根据各轴最大速度约束下
    v = min(maxfeedrate, v);
    
    while 1
        unext = u + v * Ts / norm(deboorp(2, 1:3));
        deboorpnext = DeBoorCoxNurbsCal(unext, Bsplinepath, 1);

        g2 = Tu * enlerangle2rotatemat(deboorpnext(1, 1:3), deboorpnext(1, 4:6)) / Tt;    

        theta2 = inversekinamicsDH2(g2); % 运动学逆解
        % 选解，这里采用相对于上一位置关节变化最小的点作为解
        mindis = 100;
        minindex = 1;

        for j = 1:size(theta2, 1)
            % 第二个点开始选距离上一位置最近的点
            if norm(theta2(j, :) - joint(stepnum, :)) < mindis
                minindex = j;
                mindis = norm(theta2(j, :) - joint(stepnum, :));
            end
        end

        jointnext = theta2(minindex, :); % 计算当前位置处关节角度值

        jointmid = (jointnext + joint(stepnum, :)) / 2; % 取关节位置中点

        gmid = forwardkinamicsDH(jointmid); % 计算正向运动学，得到末端位置点
        pmid = gmid(1:3, 4);

        % 计算关节位置中点到小线段的距离
        pu = g(1:3, 4);
        punext = g2(1:3, 4);
        error(stepnum) = norm(cross(pmid - pu, punext - pu)) / norm(punext - pu);
        
        if error(stepnum) < chorderror
            break;
        end
        v = v * 0.9;
        fprintf([num2str(stepnum) '\n']);
    end    
    feedratemax(stepnum, 1) = u;
    feedratemax(stepnum, 2) = v;
    
    stepnum = stepnum + 1;
end

% 将首尾两点处速度设为0
feedratemax(1, 2) = 0;
feedratemax(end, 2) = 0;

for i = length(uarr):-1:1 
    if i > 1
        if feedratemax(i - 1, 2) > feedratemax(i, 2)  % 如果第 i - 1的速度大于第i点的速度，那么计算是否需要调整
            % 先确定当前i点处允许的最大加速度
            p = scanpders(i, 1:6);      % 笛卡尔空间位置
            pd = scanpders(i, 7:12);    % 笛卡尔空间路径一阶导
            pdd = scanpders(i, 13:18);  % 笛卡尔空间路径二阶导
            jp = joint(i, :);           % 关节空间位置
            Ja = Jarr(:, :, i);         % 分析雅克比

            pdnorm = pd / norm(pd(1:3));    % 将一阶导单位化，并化为弧度制
            pdnorm(4:6) = pdnorm(4:6) * pi / 180;
            pddnorm = pdd / norm(pdd(1:3)); % 将二阶导单位化，并化为弧度制
            pddnorm(4:6) = pddnorm(4:6) * pi / 180;

            Jinv = inv(Jarr(:, :, i));  % 求第i个雅克比矩阵的逆
            
            Temp1 = zeros(6, 6);
            Temp2 = zeros(6, 6);
            for j = 1:6
                Temp1 = Temp1 + pdnorm(j) * T(:, :, i)' * Jinv' * Jder(:, :, j, i) * Jinv * T(:, :, i);
            end
            Temp2 = pdnorm(4) * TderA(:, :, i) + pdnorm(5) * TderE(:, :, i);
            
            feedtemp = feedratemax(i, 2) ^ 2 * Jinv * (Temp1 - Temp2) * pdnorm';
            
            feedtemp2(i, :) = Jinv * T(:, :, i) * pddnorm';
        end
    end
    
end

aa = 1;
