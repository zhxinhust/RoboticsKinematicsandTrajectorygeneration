function pjoint = inversekinamicsDH2(g)
% % 逆运动学

%% 这部分调试用
% clc
% clear all;
% % 
% g = forwardkinamicsDH([0, 1, -pi / 2, 0, 0, 0]);

%% 运动学逆解
% 各轴的极限位置
axis1min = -pi;
axis1max = pi;

axis2min = -75 * pi / 180;
axis2max = 95 * pi / 180;

axis3min = -246 * pi / 180;
axis3max = -10 * pi / 180;

axis4min = -2700 * pi / 180;
axis4max = 2700 * pi / 180;

axis5min = -125 * pi / 180;
axis5max = 125 * pi / 180;

axis6min = -2700 * pi / 180;
axis6max = 2700 * pi / 180;

% 机器人各连杆参数值
a1 = 400;
d0 = 830;
a2 = 1175;
a3 = 250;
d4 = 1125.33;
d6 = 230;

% 读取其次变换矩阵里的参数
nx = g(1, 1);
ny = g(2, 1);
nz = g(3, 1);

ox = g(1, 2);
oy = g(2, 2);
oz = g(3, 2);

ax = g(1, 3);
ay = g(2, 3);
az = g(3, 3);

px = g(1, 4);
py = g(2, 4);
pz = g(3, 4);

% 求第一关节，关节1的运动范围为
theta11 = atan((d6 * ay - py) / (-d6 * ax + px));
if theta11 > 0
    theta11 = theta11 - pi;
end
theta1(1, 1) = theta11;
theta12 = theta11 + pi;
theta1(2, 1) = theta12;

% 求第二关节
% 这里k1, k2, k3 都是中间要用到的变量具体参见文档
k2 = d0 + az * d6 - pz;

solutionnum = 1;

for i = 1 : length(theta1)

	% 求中间参数值
	c1 = cos(theta1(i));
	s1 = sin(theta1(i));
	
	k1 = -a1 - c1 * ax * d6 + s1 * ay * d6 + c1 * px - s1 * py;
	k3 = (a3 ^ 2 + d4 ^ 2 - a2 ^ 2 - k1 ^ 2 - k2 ^ 2) / 2 / a2;
	
    if k1 ^ 2 + k2 ^ 2 - k3 ^ 2 < 0
        % 如果出现 k1 ^ 2 + k2 ^ 2 - k3 ^ 2 < 0，那么无解，否则会出现虚数
        continue;
    end
    
    % 二轴可能存在两组解，需要进行判断，二轴运动范围为[-75, 95]，周期为2 * pi
	theta21 = atan2(k2 , k1) - atan2(k3 , sqrt(k1 ^ 2 + k2 ^ 2 - k3 ^ 2));
	theta22 = atan2(k2 , k1) - atan2(k3 , -sqrt(k1 ^ 2 + k2 ^ 2 - k3 ^ 2));
    
	theta2num = 0;
    theta21 = jointadjustinrange(theta21, axis2max, axis2min, 2 * pi); % 判断解是否在可行范围内
    if theta21 ~= 2016
        theta2(theta2num + 1) = theta21;
        theta2num = theta2num + 1;
    end
    theta22 = jointadjustinrange(theta22, axis2max, axis2min, 2 * pi);
    if theta22 ~= 2016
        theta2(theta2num + 1) = theta22;
        theta2num = theta2num + 1;
    end
    
    if theta2num == 0
        % 如果二轴所有解都不在可行范围内，则继续下次循环
        continue;
    end
    
    for tt = 1:theta2num
        
        c2 = cos(theta2(tt));
        s2 = sin(theta2(tt));

        % 求解theta3，theta3运动范围为[-246, -10]，周期为2 * pi
        theta3 = atan2(d4 * (s2 * a2 - k1) + a3 * (k2 + c2 * a2), a3 * (s2 * a2 - k1) - d4 * (k2 + a2 * c2));
        
        % 判断解出来的theta3是否可行，不可行则直接进行下次循环
        theta3 = jointadjustinrange(theta3, axis3max, axis3min, 2 * pi);
        if theta3 == 2016
            continue;
        end

        c3 = cos(theta3);
        s3 = sin(theta3);

        % 求解theta6，为了避免出现 0/0的情况，先进行判断
        theta6num = 1;
        if abs(c1 * s3 * ox - s1 * s3 * oy - c3 * oz) < 10 ^ -6 && abs(-c1 * s3 * nx + s1 * s3 * ny + c3 * nz) < 10 ^ -6
            theta61 = 0;
        else
            theta61 = -atan((c1 * s3 * ox - s1 * s3 * oy - c3 * oz) / (-c1 * s3 * nx + s1 * s3 * ny + c3 * nz));
        end
        
        % theta6运动范围为[-2700, 2700]，周期为pi
        theta62 = theta61;
        while theta62 < axis6max
            theta6(theta6num) = theta62;
            theta62 = theta62 + pi;
            theta6num = theta6num + 1;
        end

        theta62 = theta61 - pi;
        while theta62 > axis6min
            theta6(theta6num) = theta62;
            theta62 = theta62 - pi;
            theta6num = theta6num + 1;
        end

        for jj = 1:theta6num - 1
            % 根据每个theta6求解相应的 theta4 和 theta5
            s6 = sin(theta6(jj));
            c6 = cos(theta6(jj));

            % theta5运动范围为[-125, 125]，周期为2 * pi;如果不在此范围，则继续下次循环
            theta5 = atan2(-c6 * (-c1 * s3 * nx + s1 * s3 * ny + c3 * nz) - s6 * (-c1 * s3 * ox + s1 *s3 * oy + c3 * oz), -c1 * s3 * ax + s1 * s3 * ay + c3 * az);
            theta5 = jointadjustinrange(theta5, axis5max, axis5min, 2 * pi);
            if theta5 == 2016
                continue;
            end
            
            % 求解theta6，theta4运动范围为[-2700, 2700]，周期为2 * pi
            theta41 = atan2(-s6 * (c1 * c3 * nx - c3 * s1 * ny + s3 * nz) - c6 * (-c1 * c3 * ox + c3 * s1 * oy - s3 * oz), s6 * (-s1 * nx - c1 * ny) + c6 * (s1 * ox + c1 * oy));
            theta4num = 1;
            theta42 = theta41;
            while theta42 < axis4max
                theta4(theta4num) = theta42;
                theta42 = theta42 + 2 * pi;
                theta4num = theta4num + 1;
            end

            theta42 = theta41 - 2 * pi;
            while theta42 > axis4min
                theta4(theta4num) = theta42;
                theta42 = theta42 - 2 * pi;
                theta4num = theta4num + 1;
            end
            
            for kk = 1:theta4num - 1
                % 保存各组解
                pjoint(solutionnum, 1) = theta1(i);
                pjoint(solutionnum, 2) = theta2(tt);
                pjoint(solutionnum, 3) = theta3;
                pjoint(solutionnum, 4) = theta4(kk);
                pjoint(solutionnum, 5) = theta5;
                pjoint(solutionnum, 6) = theta6(jj);
                solutionnum = solutionnum + 1;
            end
        end
    end
end

%% 调试用

% for i = 1:size(pjoint, 1)
%     h(i) = max(max(abs(forwardkinamicsDH(pjoint(i, :)) - g)));
% end
% 
% max(h)