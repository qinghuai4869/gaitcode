function [pos_ic,pos_ICrelativetoFC] = fun_timeInitContact(acc,t_to,t_fc)
%UNTITLED2 初始接触检测的时刻
%   一节后向差分
    N = length(t_to);
%     atk = acc - 9.7608;
%    2023.9.13 更改 无差别
    atk = acc;
    j_win = 0.7;
    j_th = 0.95;
    Ts = 0.01;
    t_twin = [];
    point_to_fc = []; 
    jerk_tk = [];
    jerk_max=[];
%     jerk_tk =diff(atk');
%     q= length(jerk_tk);
%     for k = 1:q
%         norm_jerk(k) = norm(jerk_tk(k,:)');
%     end
    for k= 1:N
%         jerk_tk(:,k) = 1/Ts * (acc_tk(:,k+1)-acc_tk(:,k));
        t_twin(k) = j_win * t_to(k) + (1-j_win) * t_fc(k); 
        t_win(k) = fix(t_twin(k));
        %%%% 这个窗口的设定是为了啥呢
        %%%% 原因是因为最开始的脚平坦我是没算的，少1个，自然不用下一个，
        %%%%%%% 数量和脚抬起一样
        point_to_fc{k} = atk(:,t_win(k):t_fc(k));
        acc_tk = point_to_fc{k};
        M = length(acc_tk);
%         m=2;
        for m =2:M
            jerk_tk(:,m-1) = 1/Ts * (acc_tk(:,m)-acc_tk(:,(m-1)));
            norm_jerk(:,m-1) = norm(jerk_tk(:,m-1));
        end
        jerk_max(k) = max(norm_jerk);
        j_ath = j_th * jerk_max(k);
        p= find(norm_jerk >= j_ath);
        pos_ICrelativetoFC(k) = min(p(:,1));
        pos_ic(k) = pos_ICrelativetoFC(k) + t_win(k) -1 + 1;
        clear jerk_tk
        clear norm_jerk
        
        %%%%少算了一个呢！此处就在加一个了哦，或者所有的后向差分，第一个值不动，没差值
    end

end

