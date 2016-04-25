function feedratemax = NurbsScanningRobot(BSplinepath, interpolationperiod)

global Tu;  % 用户坐标系
global Tt;  % 工具坐标系
global jointinit;   % 初始点关节值

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

for u = 0:du:1
   deboorp = DeBoorCoxNurbsCal(u, BSplinepath, 2);
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
    
    Jarr(:, :, stepnum) = jacobiananalytical(joint(stepnum, :), deboorp(1, 4), deboorp(1, 5));   % 求分析雅克比矩阵
    
    pathderunit = deboorp(2, :) / norm(deboorp(2, 1:3));    % 得到单位化的速度量
    pathderunit(4:6) = pathderunit(4:6) / 180 * pi;
    
    % d(stepnum) = norm(deboorp(2, 1:3));
    
    temp = Jarr(:, :, stepnum) \ pathderunit';
    
    v = min(abs(axismaxvel) ./ abs(temp'));  % 计算根据各轴最大速度约束下
    
    v = min(maxfeedrate, v);

    
    while 1
        
        unext = u + v * Ts / norm(deboorp(2, 1:3));
    
        deboorpnext = DeBoorCoxNurbsCal(unext, BSplinepath, 1);

        g2 = Tu * enlerangle2rotatemat(deboorpnext(1, 1:3), deboorpnext(1, 4:6)) / Tt;    

        theta2 = inversekinamicsDH2(g2); % 运动学逆解
        % 选解，这里采用相对于上一位置关节变化最小的点作为接
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

for i = length(uarr):-1:1
    p = scanpders(i, 1:6);
    pd = scanpders(i, 7:12);
    pdd = scanpders(i, 13:18);
    jp = joint(i, :);
    Ja = Jarr(:, :, i);
    
    pnorm = p / norm(p(1:3));
    pdnorm = pd / norm(pd(1:3));
    
    TderA = zeros(6, 6);
    TderE = zeros(6, 6);
    
    TderA(4, 5) = -cos(p(4));
    TderA(4, 6) = -sin(p(4)) * sin(p(5));
    TderA(5, 5) = -sin(p(4));
    TderA(5, 6) = cos(p(4)) * sin(p(5));
    
    TderE(4, 6) = -cos(p(4)) * cos(p(5));
    TderE(5, 6) = sin(p(4)) * cos(p(5));
    TderE(6, 6) = -sin(p(5));
    
    T = eye(6);
    T(4:6, 4:6) = [0, -sin(p(4)), cos(p(4)) * sin(p(5));
                   0, cos(p(4)), sin(p(4)) * sin(p(5));
                   1, 0, cos(p(5))];
               
    
end

aa = 1;
