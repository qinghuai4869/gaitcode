function [r_finial] = fun_peliminatePhases(rat)
%UNTITLED5 此处显示有关此函数的摘要
%   此处显示详细说明
 %% 加速度最后信号修正
    stepsR = fun_findSteps(rat');
    re_rat = rat;
    d_zero = stepsR(:,2) - stepsR(:,1);
    N_zero = length(d_zero);
    T1_min = 18; 
    T0_min = 12;
%     T1_min = 12;
%     T0_min = 12;
    for k =1:N_zero
        if d_zero(k)<T0_min
            re_rat(1,stepsR(k,1):stepsR(k,2)) = 1;
        end
    end
    stepsR = fun_findSteps(re_rat');
    d_one = stepsR(2:end,1) - stepsR(1:end-1,2);
    N_one = length(d_one);
    for k =1:N_one
        if d_one(k) < 2*T1_min
            re_rat(1,stepsR(k,2):stepsR(k+1,1)) = 0;
        end
    end
    r_finial =re_rat;
end

