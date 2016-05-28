function IterativeCalArcLength2(startPoint, endPoint)
% 迭代计算子段弧长

global arcLength;

global simpsonVector;       % 用来保存用辛普森公式迭代计算出来的(u, l)值
global simpsonVectorIndex;  % 此变量为上数据点的数量

simpsonErr = 10^(-4);

% 利用辛普森二分迭代上半段
midPoint = (startPoint + endPoint) / 2;
subStartPoint = startPoint;
subEndPoint = midPoint;
subMidPoint = (subStartPoint + subEndPoint) / 2;

subArcLengthUpHalf = ArcLengthSimpson(subStartPoint, subMidPoint);
subArcLengthLowerHalf = ArcLengthSimpson(subMidPoint, subEndPoint);
subArcToltal = ArcLengthSimpson(subStartPoint, subEndPoint);

if (abs(subArcLengthUpHalf + subArcLengthLowerHalf - subArcToltal) < simpsonErr)
	arcLength = arcLength + subArcToltal;	% 如果满足要求则累加这半段长度
    simpsonVector(simpsonVectorIndex, 1) = subEndPoint;
    
    if simpsonVectorIndex == 1
        simpsonVector(simpsonVectorIndex, 2) = subArcToltal;
    else
        simpsonVector(simpsonVectorIndex, 2) = simpsonVector(simpsonVectorIndex - 1, 2) + subArcToltal;
    end
    simpsonVectorIndex = simpsonVectorIndex + 1;
else
	IterativeCalArcLength2(subStartPoint, subEndPoint);	%如果不满足误差要求，则继续二分进行迭代
end

% 利用辛普森二分迭代下半段
subStartPoint = midPoint;
subEndPoint = endPoint;
subMidPoint = (midPoint + endPoint) / 2;

subArcLengthUpHalf = ArcLengthSimpson(subStartPoint, subMidPoint);
subArcLengthLowerHalf = ArcLengthSimpson(subMidPoint, subEndPoint);
subArcToltal = ArcLengthSimpson(subStartPoint, subEndPoint);

if (abs(subArcLengthUpHalf + subArcLengthLowerHalf - subArcToltal) < simpsonErr)
	arcLength = arcLength + subArcToltal;	% 如果满足要求则累加这半段长度
	simpsonVector(simpsonVectorIndex, 1) = subEndPoint;
	simpsonVector(simpsonVectorIndex, 2) = simpsonVector(simpsonVectorIndex - 1, 2) + subArcToltal;
else
	IterativeCalArcLength2(subStartPoint, subEndPoint);	%如果不满足误差要求，则继续二分进行迭代
end