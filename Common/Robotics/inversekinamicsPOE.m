function pj = inversekinamicsPOE(g)
% 逆运动学

% clear all;
% 
% clc
% 
% g = forwardkinamicsPOE([0.1, 0.5, -01 / 2, 0.5, 1, 1])
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
a2 = 400;
d1 = 830;
a3 = 1175;
a4 = 250;
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

solutionnum = 0;	% 解的个数

% 求第一关节
theta1 = -atan((d6 * ay + py) / (d6 * ax + px));
if theta1 > 0
    theta1 = theta1 - pi;
end
joint(1, 1) = theta1;
theta1 = theta1 + pi;
joint(2, 1) = theta1;
solutionnum = solutionnum + 2;

% 求第二关节
% 这里k1, k2, k3 都是中间要用到的变量具体参见文档
k2 = az * d6 + pz - d1;
solutionnumpresent = 1;	% 当前解的数量，为了避免在循环内改变时引起问题
solutionnumadd = 0;
lastjoint = joint;
skipnum = 0;
for i = 1 : solutionnum
	solutionnumpresent = i + solutionnumadd;
	% 求中间参数值
	c1 = cos(lastjoint(i, 1));
	s1 = sin(lastjoint(i, 1));
	
	k1 = (ax * d6 + px) * c1 - (ay * d6 + py) * s1 - a2;
	k3 = (k1 ^ 2 + k2 ^ 2 + a3 ^ 2 - a4 ^ 2 - d4 ^ 2) / 2 / a3;
	
	if k1 ^ 2 + k2 ^ 2 - k3 ^ 2 < 0
		skipnum = i;
		continue;
	end
	
	theta21 = atan(k3 / sqrt(k1 ^ 2 + k2 ^ 2 - k3 ^ 2)) - atan(k2 / k1);
    theta22 = -atan(k3 / sqrt(k1 ^ 2 + k2 ^ 2 - k3 ^ 2)) - atan(k2 / k1);
	
	if theta21 > axis2min && theta21 < axis2max
        theta2 = theta21;
    else
        theta2 = theta22;
    end
	
	% 求θ3的值，由于θ3取值范围为[-246, -10]，而周期为π，因此可能存在两组解
	theta3 = theta2 - atan((a4 * (k1 - a3 * sin(theta2)) - d4 * (k2 - a3 * cos(theta2)))/ (d4 * (k1 - a3 * sin(theta2)) + a4 * (k2 - a3 * cos(theta2))));
    
	% 
	while theta3 < axis3min + pi / 2
		theta3 = theta3 + pi;
	end
	while theta3 > axis3max + pi / 2
		theta3 = theta3 - pi;
    end
	
	% 求cos(θ2 - θ3)和sin(θ2 - θ3)这个在求θ4，θ5，θ6需要用到
	s2m3 = (a4 * (k1 - a3 * sin(theta2)) - d4 * (k2 - a3 * cos(theta2))) / (a4 ^ 2 + d4 ^ 2);
	c2m3 = (d4 * (k1 - a3 * sin(theta2)) + a4 * (k2 - a3 * cos(theta2))) / (a4 ^ 2 + d4 ^ 2);
    
	theta6(1) = -atan(( - oz * s2m3 + (ox * c1 - oy * s1) * c2m3)/ (- nz * s2m3 + (nx * c1 - ny * s1) * c2m3));
    if theta6 > 0
        theta6(2) = theta6(1) - 2 * pi;
    else
        theta6(2) = theta6(1) + 2 * pi;
    end
    
    for jj = 1:2
        % % 求解θ4、θ5需要用到
        s6 = sin(theta6(jj));
        c6 = cos(theta6(jj));

        theta4(1) = -atan((c1 * s2m3 * (s6 * nx + c6 * ox) - s1 * s2m3 * (s6 * ny + c6 * oy) + c2m3 * (s6 * nz + c6 * oz)) / (s1 * (s6 * nx + c6 * ox) + c1 * (s6 * ny + c6 * oy)));
        if theta4(1) > 0
            theta4(2) = theta4(1) - 2 * pi;
        else
            theta4(2) = theta4(1) + 2 * pi;
        end
        theta5 = atan((c1 * c2m3 * (c6 * nx - s6 * ox) - s2m3 * (c6 * nz - s6 * oz) + s1 * c2m3 * (s6 * oy - c6 * ny)) / (-c1 * c2m3 * ax + s1 * c2m3 * ay + s2m3 * az));
        
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 1, 1) = lastjoint(i, 1);
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 1, 2) = theta2;
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 1, 3) = theta3 - pi / 2;
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 1, 4) = theta4(1);
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 1, 5) = theta5;
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 1, 6) = theta6(jj);
        
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 2, 1) = lastjoint(i, 1);
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 2, 2) = theta2;
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 2, 3) = theta3 - pi / 2;
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 2, 4) = theta4(2);
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 2, 5) = theta5;
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 2, 6) = theta6(jj);
    end
end

if skipnum == 1
    pj = pjoint(5:8, :);
elseif skipnum == 2
    pj = pjoint(1:4, :);
else
    pj = pjoint;
end

% for i = 1:size(pj, 1)
%     h = forwardkinamicsPOE(pj(i, :)) - g
% end