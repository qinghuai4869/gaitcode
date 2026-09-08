function [pos_To,pos_TorelativetoHR] = fun_timeToeOff(t_hr,t_fc,gyr)
%UNTITLED 脚趾离地时刻
%   传感器的方向独立，不必要非得对其一个轴，能够获得可靠的检测
%   tilt_rate 斜率
    w = gyr;
    N = length(t_hr);
    point_hr_fc = [];
    w_tk = [];
    tilt_rate = [];
    max_tiltrate = [];
    pos_to_half = [];
    for k=1:N
        point_hr_fc{k} = w(:,t_hr(k):t_fc(k));
        w_tk = point_hr_fc{k};
        M = length(w_tk);
        for r=1:M
            a = sum(w_tk(:,1:r),2);
            tilt_rate(:,r) = w_tk(:,r)' * a/norm(a);
        end
            if rem(M,2)==0
                m = M/2;
            else
                m = (M+1)/2;
            end
        
            max_tiltrate(k) = max(tilt_rate(1:m));
            b = 1/2 * max_tiltrate(k);
            pos_b = find(tilt_rate >= b);
            pos_first_b(k) = min(pos_b(:,1));
            new_findTiltRate = tilt_rate(pos_first_b(k):end);

            pos_newf = find(new_findTiltRate <= 0);
            if isempty(pos_newf)
                [~, valleyIndices] = findpeaks(-new_findTiltRate);
                pos_to_half(k) = min(valleyIndices);
            else 
                pos_to_half(k) = min(pos_newf);
            end
             
            % 患者出现了不到0 的情况，实际上这里只是需要转折点，后续想想
            %%%%% pos_newf(:,1)' 就一列额

            clear tilt_rate
            clear new_findTiltRate
            clear pos_b
            clear pos_newf

    end
    pos_TorelativetoHR = pos_first_b+pos_to_half-1;%%%% 相对于time_hr的位置
    pos_To = t_hr + pos_TorelativetoHR -1; %%%%%  相对于初始时刻的位置

end

