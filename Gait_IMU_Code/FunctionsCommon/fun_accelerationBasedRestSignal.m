function [r_finial] = fun_accelerationBasedRestSignal(pre_tk,h,N,th_final)
%UNTITLED5 此处显示有关此函数的摘要
%   此处显示详细说明
    atk_pre = abs(pre_tk);
    acc_ath_up =(1 + h) * th_final;
    acc_ath_down =(1 - h) * th_final;
    ra_ = zeros(1,N);
    for k=2:N
        if atk_pre(k) > acc_ath_up
            ra_(k) = 1;

        else
            if atk_pre(k) < acc_ath_down
            ra_(k) = 0;
            else
                ra_(k) = ra_(k-1);
            end
        end 
    end
    rat_ = zeros(1,N);
    rat_(N) = ra_(N); 
    for k=N-1:-1:1
        if ra_(k) == 1
            rat_(k) = 1;
        else
            if atk_pre(k) < acc_ath_down
                rat_(k) = 0;
            else
                rat_(k)=rat_(k+1);
            end
        end
    end
    %% 加速度最后信号修正
    stepsR = fun_findSteps(rat_');
    re_rat = rat_;
    d_zero = stepsR(:,2) - stepsR(:,1);
    N_zero = length(d_zero);
%     d_one = stepsR(2:end,1) - stepsR(1:end-1,2);
%     N_one = length(d_one);
    T1_min = 18;
    T0_min = 12;
%     T1_min = 12;    
%     T0_min = 12;
    for k =1:N_zero
        if d_zero(k) < T0_min
            re_rat(1,stepsR(k,1):stepsR(k,2)) = 1;
        end
    end
    stepsR = fun_findSteps(re_rat');
    d_one = stepsR(2:end,1) - stepsR(1:end-1,2);
    N_one = length(d_one);

    for k =1:N_one
        if d_one(k) < T1_min
            re_rat(1,stepsR(k,2):stepsR(k+1,1)) = 0;
        end
    end
    r_finial =re_rat;
end

