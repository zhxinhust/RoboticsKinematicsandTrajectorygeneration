function [subSegmentUPara, subSegmentFeedPeakSmall, subSegmentFeedPeakBig] = NurbsScanning(dualquatpath)
% 进行速度极值扫描，得到分段信息及各段的速度最大最小值以及相应分段的u的值

% 插补频率，及插补周期Ts = 0.5ms
global interpolationFrequence;
interpolationFrequence = 500;
global interpolationPeriod;
interpolationPeriod = 1 / interpolationFrequence;

Ts = interpolationPeriod;			% 插补周期

global chordErr;
chordErr = 0.005;	% 允许最大轮廓误差

global maxAcc;
global maxJerk;

maxAcc = 50;      % 允许的最大加速度
maxJerk = 800;    % 允许的最大跃度

global blockFeed;
blockFeed = 100; 	% 本段进给速度

curvatureNurbsPreLast = 0;	% 上上次的曲率值;
curvatureNurbsLast = 0; 	% 上次的曲率值
curvatureNurbs = 0;			% 曲率值
lastFiveStepFeed = zeros(1, 4);	% 前面四步时的速度，用来使用5步法求速度导矢
 
oneSubSegmentFlag = 0;	% 只有一个子段的标志位
subSegmentMaxFeed = 0;	% 子段内最大速度；

uNurbs = 0;	% 曲线参数
 
cenAccLimitedCurvature = maxAcc / blockFeed^2;	% 向心加速度设定的曲率阈值

jerkLimitedCurvature = sqrt(maxJerk / blockFeed^3);	% 跃度设定的曲率值

% 取几个限制的最小值;
curvatureLimitMin = min([1, cenAccLimitedCurvature, jerkLimitedCurvature]);

subSegmentNum = 2;
stepNumber = 1;

while uNurbs < 1
    % 计算型值点及一二三阶导矢

    deboorp = DeBoorCoxNurbsCal(uNurbs, dualquatpath, 2);

    % 求型值点
    knotCor(stepNumber, :) = deboorp(1, :);
    	
    % 更新保存的曲率值;
    curvatureNurbsPreLast = curvatureNurbsLast;
	curvatureNurbsLast = curvatureNurbs;
    
    % 求当前点的曲率
	curvatureNurbs = norm(cross(deboorp(2, 1:3), deboorp(3, 1:3))) / norm(deboorp(2, 1:3))^3;                        % 根据曲率公式求曲率
    
    curvetureArr(stepNumber, 1) = uNurbs;
    curvetureArr(stepNumber, 2) = curvatureNurbs;
    
    curvatureRadiusNurbs = 1 / curvatureNurbs;	% 计算曲率半径
	
	% 根据Ts，曲率半径，弓高误差，进给速度，kcbc，等计算Vaf，Vcbf，确定V(ui)
	chordLimitedFeed = 2 / Ts * sqrt(curvatureRadiusNurbs ^ 2 - (curvatureRadiusNurbs - chordErr)^2);
	
	adaptiveAlgorithmFeed = min([blockFeed, chordLimitedFeed]);     % Vaf
	curvatureAlgorithmFeed = 1 * blockFeed / (curvatureNurbs + 1);	% Vcbf
	
	maxAccLimitedFeed = sqrt(maxAcc / curvatureNurbs);					% 计算加速度速度约束	
	maxJerkLimitedFeed = (maxJerk / curvatureNurbs^2)^(1 / 3);	% 计算跃度速度约束
	
	% 得到Vaf, Vbf, F 以及加速度、跃度约束下的速度最小值
	currentStepFeed = min([adaptiveAlgorithmFeed, curvatureAlgorithmFeed, blockFeed, maxAccLimitedFeed, maxJerkLimitedFeed]);
	
    feedLimitArr(stepNumber) = currentStepFeed;
    
	% 利用5步法求速度导数
	currentStepFeedDer1 = 1 / (12 * Ts) * (3 * lastFiveStepFeed(1) - 16 * lastFiveStepFeed(2) + 36 * lastFiveStepFeed(3) - 48 * lastFiveStepFeed(4) + 25 * currentStepFeed);
	% 更新保存的前几步速度
	lastFiveStepFeed = [lastFiveStepFeed(2:4), currentStepFeed];
	
	% 确定下一步步长，并保存当前参数u
	uNurbsLast = uNurbs;
% 	uNurbs = uNurbs + (currentStepFeed * Ts + Ts^2 * currentStepFeedDer1 / 2) / norm(deboorp(2, 1:3)) - (currentStepFeed * Ts)^2 * (dot(deboorp(2, 1:3), deboorp(3, 1:3))) / (2 * (norm(deboorp(2, 1:3)))^4);
	
	if (uNurbs > uNurbsLast) && (uNurbs > 0.001)
		if (curvatureNurbsLast > curvatureNurbs) && (curvatureNurbsLast > curvatureNurbsPreLast)
			% 如果都不满足曲率阈值条件，那么将曲线作为一个子段处理，保存相应的数据；
			if curvatureNurbsLast >= curvatureLimitMin
				oneSubSegmentFlag = 1;
				subSegmentUPara(subSegmentNum) = uNurbsLast;
				subSegmentFeedPeakBig(subSegmentNum) = subSegmentMaxFeed;
				subSegmentFeedPeakSmall(subSegmentNum) = lastStepFeed;
                splitPoints(subSegmentNum, :) = knotCor(stepNumber, :);     % 保存分段点坐标，方便查看用
				subSegmentNum = subSegmentNum + 1;
				subSegmentMaxFeed = 0;
			end
		end
	end	
	% 更新子段内最大速度；
	subSegmentMaxFeed = max(subSegmentMaxFeed, currentStepFeed);
	% 保存当前速度值；
	lastStepFeed = currentStepFeed;
    
    uNurbs = uNurbs + 0.0001;      % 测试用，不适用预插补，直接等距累加参数u
    stepNumber = stepNumber + 1;
end

% figure;
% % 绘制对偶四元数求得的曲率曲线
% plot(curvetureArr(:, 1), curvetureArr(:, 2));
% % readCurvature = importdata('.\data\inputdata\Curvature.txt');
% % hold on;
% % plot(readCurvature(:, 1), readCurvature(:, 2), 'r');
% set(gca, 'fontsize', 25);
% title('Curvature');
% ylim([0 5]);
% hold on;
% % 绘制用VC程序求得的曲率曲线
% curvatureVs = importdata('Curvature.txt');
% plot(curvatureVs(:, 1), curvatureVs(:, 2), 'r');
% % 绘制用Matlab直接对刀尖点插值求得的曲率曲线
% load('interpolateCurvature.mat');
% plot(curvatureArr(:, 1), curvatureArr(:, 2), 'g');


if oneSubSegmentFlag == 1
	% 最初子段的起始速度为0 
	subSegmentFeedPeakSmall(1) = 0;
	% 最末子段的终止速度为0
	subSegmentUPara(subSegmentNum) = 1;
	subSegmentFeedPeakSmall(subSegmentNum) = 0;
	subSegmentFeedPeakBig(subSegmentNum) = subSegmentMaxFeed;
else
	% 如果没有速度拐点，那么初始速度和终止速度均为0，并且最大速度为整段的最大速度
	subSegmentFeedPeakSmall(1) = 0;
	subSegmentUPara(subSegmentNum) = 1;
	subSegmentFeedPeakSmall(subSegmentNum) = 0;
	subSegmentFeedPeakBig(2) = subSegmentMaxFeed;
end

% figure;
% plot3(knotCor(:, 1), knotCor(:, 2), knotCor(:, 3));
% hold on;
% plot3(splitPoints(:, 1), splitPoints(:, 2), splitPoints(:, 3), '*r');
% title('split points', 'fontsize', 24);
% set(gca, 'fontsize', 25);