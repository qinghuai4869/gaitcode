function [ath_final,norm_imu] = fun_autoThreshold(~,N,w,norm_imu)
%UNTITLED3 此处显示有关此函数的摘要
%   ~ 为原本为time，不需要了，就去掉了
  
    %% anto threshold adataption
    % 初始阈值
    max_ath  = max(norm_imu);
    min_ath  = min(norm_imu);
    ath_init = 1/2 * (max_ath + min_ath);
%     the_ =[];
%     the_ =[time;norm_imu];
    % 迭代次数 M
    M = 200;
    ath = zeros(1,M);
    ath(1)=ath_init;
    
    for m= 1:M
%         T_up = zeros(1,N);
%         T_down = zeros(1,N);
        ath_up   = zeros(1,N);
        ath_down = zeros(1,N);
        T_numbera_up   = 0;
        T_numbera_down = 0;
        for k= 1:N
            if norm_imu(k) > ath(m)
%                 T_up(k) = time(k);
                T_numbera_up = T_numbera_up + 1;
                ath_up(k) = norm_imu(k);
            else
%                 T_down(k) = time(k);
                T_numbera_down = T_numbera_down + 1;
                ath_down(k) = norm_imu(k);
            end
        end
%         T_up_sum =sum(T_up);
%         T_down_sum =sum(T_down);
        ath_up_sum   = sum(ath_up);
        ath_down_sum = sum(ath_down);
        w_b = 1 - w;
%         ath(m+1) = w/abs(T_down_sum) * ath_down_sum + w_b/abs(T_up_sum)*ath_up_sum; 
        ath(m+1) = w/abs(T_numbera_down) * ath_down_sum + w_b/abs(T_numbera_up)*ath_up_sum; 
    end 
%     times_ath = tabulate(ath);
%     a_th_p =find(times_ath(:,2)>10);
%     ath_final= times_ath(a_th_p);
    ath_final=ath(201);
    % 判断迭代是否充分
    % if times_ath(:,2)>10
    %     a_th_p =find(times_ath(:,2)>10);
    %     ra= times_ath(a_th_p);
    % else
    %     n=m+20;
    % end
%     if ath_final < 1.8
%          ath_final = 1.8;
%     end
%         
end

