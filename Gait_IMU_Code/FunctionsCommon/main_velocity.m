%%% ∑ΩœÚ’¶«Û£ø
function [v,pos] = main_velocity(acc,t_rest)
Ts = 0.01;
% g = 9.7860;
g = 9.81;
g_ = [0,0,g]';
a_l = acc'-g_;
pos_timerest = diff(t_rest);
N = length(pos_timerest);
a_timerest = [];
for k =1:N
    a_timerest{k} = a_l(:,t_rest(k):t_rest(k+1));
    a_tk = a_timerest{k};
    M = length(a_tk);
    v_tk = zeros(3,M);
    vdf_tk = zeros(3,M);
    pos_tk = zeros(3,M);
    for m = 1:M
        v_tk(:,m) = Ts * sum(a_tk(:,1:m),2);
    end
%     v_tk1  = v_tk - v_tk(:,1);
    v_tk1  = v_tk;
    for m = 1:M
        b = (m-1)/(M-1);
        
        vdf_tk(:,m) = v_tk1(:,m) - b * v_tk1(:,M);
        pos_tk(:,m) = Ts * sum(vdf_tk(:,1:m),2);
    end
    v{k} = vdf_tk; 
    pos{k} =pos_tk;
end
end