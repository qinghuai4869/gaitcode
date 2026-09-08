function [stepR,stepL] = fun_stepsLength(pos_right,pos_left,steps_r,steps_L)
%UNTITLED2 步长，验证行走方向的轴！
%   均值还是？默认左脚先走
N = length(steps_r);
%%% 左
for k = 1:N-1
    stepL1(k) = mean(pos_left(steps_L(k+1,1):steps_L(k+1,2),1)) - mean(pos_right(steps_r(k,1):steps_r(k,2),1));
end
for k = 2:N-1
    stepR1(k-1) = mean(pos_right(steps_r(k,1):steps_r(k,2),1)) - mean(pos_left(steps_L(k,1):steps_L(k,2),1));
end


% v = mean(steps_a(:,2) - steps_f(:,2));
v = mean(steps_r(1:3,2) - steps_L(1:3,2));
%%%%%% 判断第一步迈出得是左脚还是右脚
if v > 0
    stepR = abs(stepR1);
    stepL = abs(stepL1);
else 
    if v < 0
        stepR = abs(stepL1);
        stepL = abs(stepR1);
    else
        disp('input error \n please retype ')
end
end

