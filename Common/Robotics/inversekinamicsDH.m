function pj = inversekinamicsDH(g)
% % 逆运动学

% clc
% clear all;
% % 
% g = forwardkinamicsDH([0, 1, -pi / 2, 0, 0, 0]);

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

solutionnum = 0;	% 解的个数


% 求第一关节
theta1 = atan2((d6 * ay - py) , (-d6 * ax + px));
if theta1 > 0
    theta1 = theta1 - pi;
end
joint(1, 1) = theta1;
theta1 = theta1 + pi;
joint(2, 1) = theta1;
solutionnum = solutionnum + 2;

% 求第二关节
% 这里k1, k2, k3 都是中间要用到的变量具体参见文档
k2 = d0 + az * d6 - pz;
solutionnumpresent = 1;	% 当前解的数量，为了避免在循环内改变时引起问题
solutionnumadd = 0;
lastjoint = joint;
skipnum = 0;
for i = 1 : solutionnum
	solutionnumpresent = i + solutionnumadd;
	% 求中间参数值
	c1 = cos(lastjoint(i, 1));
	s1 = sin(lastjoint(i, 1));
	
	k1 = -a1 - c1 * ax * d6 + s1 * ay * d6 + c1 * px - s1 * py;
	k3 = (a3 ^ 2 + d4 ^ 2 - a2 ^ 2 - k1 ^ 2 - k2 ^ 2) / 2 / a2;
	
	% 二轴运动范围为：[-75, 95],不足180, 因此只有一个解。
    if k1 ^ 2 + k2 ^ 2 - k3 ^ 2 < 0
        % 如果出现 k1 ^ 2 + k2 ^ 2 - k3 ^ 2 < 0，那么无解，否则会出现虚数
        skipnum = i;    % 记录下来是在哪里跳过了，这样可以修正得到的借
        continue;
    end
    
	theta21 = atan2(k2 , k1) - atan2(k3 , sqrt(k1 ^ 2 + k2 ^ 2 - k3 ^ 2));
	theta22 = atan2(k2 , k1) + atan2(k3 , sqrt(k1 ^ 2 + k2 ^ 2 - k3 ^ 2));
    
    if theta21 > axis2min && theta21 < axis2max
        theta2 = theta21;
   % elseif theta22 > axis2min && theta22 < axis2max
    else
        theta2 = theta22;
    end
	
	joint(solutionnumpresent, 2) = theta2;
    c2 = cos(theta2);
    s2 = sin(theta2);
	
    theta3 = atan2(d4 * (s2 * a2 - k1) + a3 * (k2 + c2 * a2), a3 * (s2 * a2 - k1) - d4 * (k2 + a2 * c2));
% 	theta3 = atan2((-c2 * a2 - k2) * d4 + (s2 * a2 - k1) * a3, (-c2 * a2 - k2) * a3 - (s2 * a2 - k1) * d4);
	
	% k4 = (k1 ^ 2 + k2 ^ 2 - d4 ^ 2 - a2 ^ 2 - a3 ^ 2) / 2 / a2;
	
	% theta2p31 = atan2(k4, sqrt(a3 ^ 2 + d4 ^ 2 - k4 ^ 2)) - atan2(a3, d4);
	% theta2p32 = -atan2(k4, sqrt(a3 ^ 2 + d4 ^ 2 - k4 ^ 2)) - atan2(a3, d4);
	
	% 求θ3的值，由于θ3取值范围为[-246, -10]，而周期为2 * π，因此可能存在1组解
	% theta3 = atan2((-d4 * (k2 + a2 * c2) + a3 * (k1 - a2 * s2)) , (d4 * (k1 - a2 * s2) + a3 * (k2 + a2 * c2)));
	% 
	while theta3  < axis3min
		theta3 = theta3 + 2 * pi;
    end
    
	while theta3 > axis3max
		theta3 = theta3 - 2 * pi;
    end
	
% 	% 求cos(θ2 - θ3)和sin(θ2 - θ3)这个在求θ4，θ5，θ6需要用到
% 	s2m3 = (a3 * k1 - d4 * k2 - a2 * (sin(theta2) * a3 + cos(theta2) * d4)) / (a3 ^ 2 + d4 ^ 2);
% 	c2m3 = -(k1 * d4 + k2 * a3 - a2 * (-cos(theta2) * a3 + sin(theta2) * d4)) / (a3 ^ 2 + d4 ^ 2);
    c3 = cos(theta3);
    s3 = sin(theta3);
    
    if abs(c1 * s3 * ox - s1 * s3 * oy - c3 * oz) < 10 ^ -6 && abs(-c1 * s3 * nx + s1 * s3 * ny + c3 * nz) < 10 ^ -6
        theta6(1) = 0;
        theta6(2) = 2 * pi;
    else
        theta6(1) = -atan((c1 * s3 * ox - s1 * s3 * oy - c3 * oz) / (-c1 * s3 * nx + s1 * s3 * ny + c3 * nz));
        if theta6(1) > 0
            theta6(2) = theta6(1) - pi;
        else
            theta6(2) = theta6(1) + pi;
        end
    end

	
    for jj = 1:2
        % 由于要除以sinθ6，因此为了避免奇异，先进行大小判断
        s6 = sin(theta6(jj));
        c6 = cos(theta6(jj));

        theta4(1) = atan2(-s6 * (c1 * c3 * nx - c3 * s1 * ny + s3 * nz) - c6 * (-c1 * c3 * ox + c3 * s1 * oy - s3 * oz), s6 * (-s1 * nx - c1 * ny) + c6 * (s1 * ox + c1 * oy));
% 		theta4(1) = atan2(-s6 * (c1 * s3 * nx - s1 * s3 * ny - c3 * nz) - c6 * (-c1 * s3 * ox + s1 * s3 * oy + c3 * oz), s6 * (-s1 * nx - c1 * ny) + c6 * (s1 * ox + c1 * oy));
        % theta4(1) = -atan2((s6 * (c1 * s3 * nx - s1 * s3 * ny - c3 * nz) + c6 * (-c1 * s3 * ox + s1 * s3 * oy + c3 * oz)) , (s6 * (-s1 * nx - c1 * ny) + c6 * (s1 * ox + c1 * oy)));
%         theta4(1) = -atan((c1 * s2m3 * s6 * nx - s1 * s2m3 * s6 * ny + c2m3 * s6 * nz + c1 * s2m3 * c6 * ox - s1 * s2m3 * c6 * oy + c2m3 * c6 * oz) / (s1 * s6 * nx + c1 * s6 * ny + c6 * (s1 * ox + c1 * oy)));
        if theta4(1) > 0
            theta4(2) = theta4(1) - 2 * pi;
        else
            theta4(2) = theta4(1) + 2 * pi;
        end
%         theta5 = -atan((-c1 * c2m3 * c6 * nx + c2m3 * c6 * s1 * ny + s2m3 * c6 * nz + c1 * c2m3 * s6 * ox - s1 * c2m3 * s6 * oy - s2m3 * s6 * oz) / (-c1 * c2m3 * ax + c2m3 * s1 * ay + s2m3 * az));
        % theta5 = -atan2((c6 * (c1 * c3 * nx - c3 * s1 * ny + s3 * nz) + s6 * (c1 * c3 * ox - c3 * s1 * oy + s3 * oz)) , (c1 * c3 * ax - c3 * s1 * ay + s3 * az));
        theta5 = atan2(-c6 * (-c1 * s3 * nx + s1 * s3 * ny + c3 * nz) - s6 * (-c1 * s3 * ox + s1 *s3 * oy + c3 * oz), -c1 * s3 * ax + s1 * s3 * ay + c3 * az);
% 		theta5 = atan2(-c6 * (c1 * c3 * nx - s1 * c3 * ny + s3 * nz) - s6 * (c1 * c3 * ox - s1 * c3 * oy + s3 * oz), c1 * c3 * ax - s1 * c3 * ay + s3 * az);
		
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 1, 1) = lastjoint(i, 1);
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 1, 2) = theta2;
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 1, 3) = theta3;
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 1, 4) = theta4(1);
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 1, 5) = theta5;
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 1, 6) = theta6(jj);
        
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 2, 1) = lastjoint(i, 1);
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 2, 2) = theta2;
        pjoint((i - 1) * 4 + (jj - 1) * 2 + 2, 3) = theta3;
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
%     h = forwardkinamicsDH(pj(i, :)) - g
% end