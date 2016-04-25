function [interpCor, sVelProfilePlan] = NurbsInterpProcess(feedSmall, feedBig, accT, decT, contT, finT, bsplinepath)
% 插补过程

global interpolationFrequence;

global maxJerk;

global CP;

maxJerkmode2 = maxJerk / 2;

subNum = length(accT);
tempLoopIndex = 0;
subSegmentInterpIndex = 1;

deBoorP = zeros(4, 6);
deBoorPLast = zeros(4, 6);
deBoorPLast(1, :) = CP(1, :); 

lastStepFeed = 0;
lastStepAcc = 0;
uNurbs = 0;

% type5VelDeltaCv = 0;

while tempLoopIndex <= round(finT(subNum))
    if tempLoopIndex == 0
        uNurbs = 0;
        uNurbsLast = 0;
        Vstr = feedSmall(subSegmentInterpIndex);
        Vend = feedSmall(subSegmentInterpIndex + 1);
        
        % 获取各个阶段时间
        subAccT = accT(subSegmentInterpIndex) / interpolationFrequence;
        subDecT = decT(subSegmentInterpIndex) / interpolationFrequence;
        subConT = contT(subSegmentInterpIndex) / interpolationFrequence;
        
        % 获取时间编号
        subAccTNum = accT(subSegmentInterpIndex);
        subDecTNum = decT(subSegmentInterpIndex);
        subConTNum = contT(subSegmentInterpIndex);
        
        Vmax = Vstr + maxJerk * subAccT^2;
        type1VelDeltaCv = Vstr;
        type2VelDeltaCv = Vstr - maxJerk * subAccT^2;
        type4VelDeltaCv = Vstr + maxJerk * subAccT^2;
        type5VelDeltaCv = Vend;
		
        maxJerkMulAccTimeMul2 = 2 * maxJerk * subAccT;
        accAndCvTimeSum = 2 * subAccT + subConT;
        segTimeSum = accAndCvTimeSum + 2 * subDecT;
    end
    
    while tempLoopIndex >= finT(subSegmentInterpIndex)
        subSegmentInterpIndex = subSegmentInterpIndex + 1;
		
        if subSegmentInterpIndex + 1 > length(feedSmall)
            break;
        end
        
		Vstr = feedSmall(subSegmentInterpIndex);
        Vend = feedSmall(subSegmentInterpIndex + 1);
		Vmax = feedBig(subSegmentInterpIndex + 1);
		
		% 获取各个阶段时间
        subAccT = accT(subSegmentInterpIndex) / interpolationFrequence;
        subDecT = decT(subSegmentInterpIndex) / interpolationFrequence;
        subConT = contT(subSegmentInterpIndex) / interpolationFrequence;
        
        % 获取时间编号
        subAccTNum = accT(subSegmentInterpIndex);
        subDecTNum = decT(subSegmentInterpIndex);
        subConTNum = contT(subSegmentInterpIndex);
        
        type1VelDeltaCv = Vstr;
        type2VelDeltaCv = Vstr - maxJerk * subAccT^2;
        type4VelDeltaCv = Vstr + maxJerk * subAccT^2;
        type5VelDeltaCv = Vend;
		
        maxJerkMulAccTimeMul2 = 2 * maxJerk * subAccT;
        accAndCvTimeSum = 2 * subAccT + subConT;
        segTimeSum = accAndCvTimeSum + 2 * subDecT;
    end
	
	if subSegmentInterpIndex == 1
		addTemp = 0;	
	else
		addTemp = finT(subSegmentInterpIndex - 1);
	end

	deltaSubSegmentTime = (tempLoopIndex - addTemp) / interpolationFrequence;

	if tempLoopIndex <= subAccTNum + addTemp
		% 说明处于加加速阶段
		currentStepFeed = type1VelDeltaCv + maxJerkmode2 * deltaSubSegmentTime^2;
		currentStepAcc = maxJerk * deltaSubSegmentTime;
	elseif tempLoopIndex <= 2 * subAccTNum + addTemp			%判断是否到达加减速时间末尾
		% 如果没有到达，则处于加减速阶段
		currentStepFeed = type2VelDeltaCv - maxJerkmode2 * deltaSubSegmentTime^2 + maxJerkMulAccTimeMul2 * deltaSubSegmentTime;
		currentStepAcc = maxJerkMulAccTimeMul2 - maxJerk * deltaSubSegmentTime;
	elseif tempLoopIndex <= 2 * subAccTNum + subConTNum + addTemp		%判断是否达到匀速时间末尾

		% 如果没有达到，则匀速运行
		currentStepFeed = Vmax;
		currentStepAcc = 0;
	elseif tempLoopIndex <= 2 * subAccTNum + subConTNum + subDecTNum + addTemp		% 判断是否到达减减速运行时间末尾
		% 如果没有到达，则减加速运行
		currentStepFeed = type4VelDeltaCv - maxJerkmode2 * (deltaSubSegmentTime - accAndCvTimeSum)^2;
		currentStepAcc = maxJerk * (accAndCvTimeSum - deltaSubSegmentTime);
	else
		currentStepFeed = type5VelDeltaCv + maxJerkmode2 * (segTimeSum - deltaSubSegmentTime)^2;
		currentStepAcc = maxJerk * (deltaSubSegmentTime - segTimeSum);
	end

	sVelProfilePlan(tempLoopIndex + 1) = currentStepFeed;
	
	% 采用泰勒二阶展开求下一步参数u
    uNurbs = SecondTalorForInterp(uNurbs, lastStepFeed, lastStepAcc, deBoorP, tempLoopIndex);
	
	if uNurbs > 1
		break;
	end
	
	% 计算型值点及一二三阶导矢
	deBoorP = DeBoorCoxNurbsCal(uNurbs, bsplinepath, 2);
    % 计算插补点处的曲率及弓高误差
    [interpStepCurvature(tempLoopIndex + 1), interpStepChordErr(tempLoopIndex + 1)] = CurrentStepCurvature(currentStepFeed, deBoorP);
	% 计算速度约束值
	feedLimit(tempLoopIndex + 1) = FeedLimitCal(interpStepCurvature(tempLoopIndex + 1));
    % 计算速度波动值
%     [actlFeedrate(tempLoopIndex + 1), actlFeedrateFluctuateErr(tempLoopIndex + 1), actlFeedrateFluctuateErrZhu(tempLoopIndex + 1)] = ...
%         FeedFluctuationActlAndPredicted(currentStepFeed, lastStepFeed, interpStepCurvature(tempLoopIndex + 1), deBoorP, deBoorPLast);
    
    % 保存这次计算的数据
    lastStepFeed = currentStepFeed;
    lastStepAcc = currentStepAcc;
    lastStepCurvature = interpStepCurvature(tempLoopIndex + 1);
    
    deBoorPLast = deBoorP;
    
    interpCor(tempLoopIndex + 1, :) = deBoorP(1, :);
    
    uParaVector(tempLoopIndex + 1) = uNurbs;
    
    tempLoopIndex = tempLoopIndex + 1;
end

x = 50;
y = 50;
width = 480;
height= 270;

fontsize = 15;
fontsizelabel = 15;
figposition = [50, 50, 1200, 900] / 2;

figure('Position',figposition);
plot((1:length(sVelProfilePlan)) / interpolationFrequence ,sVelProfilePlan, 'r', 'linewidth', 1.5);
hold on;
plot((1:length(feedLimit)) / interpolationFrequence , feedLimit, 'b', 'linewidth', 1.5);
set(gca, 'fontsize', fontsize);
ylim([0, 130]);
ylabel('Feedrate (mm/s)', 'fontsize', fontsizelabel);
xlabel('Time (s)', 'fontsize', fontsizelabel);
h2 = legend('速度规划曲线', '速度极值曲线');
% plot(actlFeedrate, 'r');

% figure;
% plot(interpCor(:, 1), interpCor(:, 2), '*');


