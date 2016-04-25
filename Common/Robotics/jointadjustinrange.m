function thetad = jointadjustinrange(theta, thetamax, thetamin, cyc)
% 根据此轴的运动范围及周期，进行调整位置，如果不在此范围内，则返回2016表示无解

while theta > thetamax
    theta = theta - cyc;
end

while theta < thetamin
    theta = theta + cyc;
end

if theta < thetamax && theta > thetamin
    thetad = theta;
else
    thetad = 2016;
end