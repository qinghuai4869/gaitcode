function [stepR,stepL] = fun_stepWidthLength(pos_right,pos_left,steps_r,steps_L)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
% 给定两个点 p1，p2，它们可以生成一条直线 p1p2 = p1 - p2
% 现在有一条直线外的点 p0
% 上述三个点都用向量表示到原点，即 p = [px, py]T
% 因此，点到直线的前进距离公式可以表示为：
% （注意三个点的坐标都用列向量表示）
N = min(length(steps_r),length(steps_L));

for k = 1:N
%     L1(k,:) = mean(pos_left(steps_L(k,1):steps_L(k,2),:));%%% 取所有触地的脚的位置
%     R1(k,:) = mean(pos_right(steps_r(k,1):steps_r(k,2),:));
%     L1(k,:) = mean(pos_left(steps_L(k,1)+10:steps_L(k,2)-10,:));%%% 取所有触地的脚的位置
%     R1(k,:) = mean(pos_right(steps_r(k,1)+10:steps_r(k,2)-10,:));
  

%%% 之前是取减少前后20个数据点，现在改为减少前后10%。
%%% 左边
    data_left = pos_left(steps_L(k,1):steps_L(k,2),:);
       % 计算数据长度
    data_Leftlength = length(data_left);
    
    % 计算10%的位置 取整
    ten_percent_index_left = round(0.1 * data_Leftlength);
    
    % 取中间80%的数据
    middle_LeftData = data_left(ten_percent_index_left +1 : data_Leftlength-ten_percent_index_left,:);

%%%%% 右边
    data_right = pos_right(steps_r(k,1):steps_r(k,2),:);
       % 计算数据长度
    data_Rightlength = length(data_right);
    
    % 计算10%的位置 取整
    ten_percent_index_Right = round(0.1 * data_Rightlength);
    
    % 取中间80%的数据
    middle_RightData = data_right(ten_percent_index_Right+1 : data_Rightlength-ten_percent_index_Right,:);

    L1(k,:) = mean(middle_LeftData);%%% 取所有触地的脚的位置
    R1(k,:) = mean(middle_RightData);


end
%%%% 左右脚先行需要判断
v = mean(steps_r(1:3,2) - steps_L(1:3,2));
if v > 0
    R = R1;
    L = L1;
else 
    if v < 0
        R = L1;
        L = R1;
    else
        disp('input error \n please retype ')
    end
end
%% 左脚步长（第一步）

% p0 = [p0x; p0y];
% p1 = [p1x; p1y];
% p2 = [p2x; p2y];
% for k = 2:N-1(存在奇偶问题)
for k = 2:N 
    Lp0 = L(k,:)';
    Rp1 = R(k-1,:)';
    Rp2 = R(k,:)';

    % 计算点到直线的垂足
    foot = Rp1 + (Lp0 - Rp1)' * (Rp2 - Rp1) / norm(Rp2 - Rp1)^2 * (Rp2 - Rp1);

    % 计算点到垂足的距离
    stepL1(k-1) = norm(Rp1 - foot);
    
end
%% 右脚步长（第二步）
for k = 2:N-1
    Rp0 = R(k,:)';
    Lp1 = L(k,:)';
    Lp2 = L(k+1,:)';


    % 计算点到直线的垂足
    foot = Lp1 + (Rp0 - Lp1)' * (Lp2 - Lp1) / norm(Lp2 - Lp1)^2 * (Lp2 - Lp1);

    % 计算点到垂足的距离
    stepR1(k-1) = norm(Lp1 - foot);
end

%%  
% v = mean(steps_a(:,2) - steps_f(:,2));
% v = mean(steps_r(1:3,2) - steps_L(1:3,2));
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

%     stepR = abs(stepR1);
%     stepL = abs(stepL1);

end

