function arcLengthVector = NurbsSubSegArcLenCal(subSegmentUPara, dualquatpath)
% 利用辛普森公式+二分迭代求每段弧长
% 输出的arcLengthVector 的前面为每子段的长度，最后一个数据为刀路总长

global KnotVector;  % 节点向量
global CP;      % 控制点
global curveDegree; % 曲线阶数
global p0;
global V0;

global simpsonVectorIndex;  % 此变量为上数据点的数量
simpsonVectorIndex = 1;

KnotVector = dualquatpath.knotvector;
CP = dualquatpath.controlp;
curveDegree = dualquatpath.splineorder;

if size(CP, 2) == 8
    p0 = dualquatpath.tip0;
    V0 = dualquatpath.vector0;
end

% 定义全局变量，迭代计算子段弧长时用
global arcLength;
arcLength = 0;

arcLengthWhole = 0;

% 子段数
subSegmentNum = length(subSegmentUPara) - 1;

% 初始化
arcLengthVector = zeros(1, subSegmentNum + 1);

for tempLoopIndex = 1:subSegmentNum
    
    %%
    
	% 初始化个字段长度计算需用到的值
    startPoint = subSegmentUPara(tempLoopIndex);
    endPoint = subSegmentUPara(tempLoopIndex + 1);
	
	arcLength = 0;	% 总长度清零，以进行下一次迭代
    
    IterativeCalArcLength2(startPoint, endPoint);	% 迭代计算子段弧长
	
	arcLengthVector(tempLoopIndex) = arcLength;		% 保存子段弧长
	
	arcLengthWhole = arcLengthWhole + arcLength;	% 累加总长度
end

arcLengthVector(subSegmentNum + 1) = arcLengthWhole;