function interp = linearconstspeedinterp(pathdata, F, Ts)
% 对给定的路径进行匀速插补

len = zeros(1, size(pathdata, 1) - 1);
for i = 1:size(pathdata, 1) - 1
    len(i) = norm(pathdata(i + 1, 1:3) - pathdata(i, 1:3));
end

interpindex = 1;
for i = 1:size(pathdata, 1) - 1
    k = 1;
    s = 0;
    while s < len(i)
        s = F * k * Ts;
        interp(interpindex, :) = pathdata(i, :) + s / len(i) * (pathdata(i + 1, :) - pathdata(i, :));
        k = k + 1;
        interpindex = interpindex + 1;
    end
end

% for i = 1:size(interp, 1) - 1
%     dis(i) = norm(interp(i + 1, 1:3) - interp(i, 1:3));
% end
% 
% aa = 0;
