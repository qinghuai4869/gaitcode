function [footsteplength] =stridelength(position,step)
    n=length(step);
    p_footcontact = zeros(n,2);
    footsteplength = zeros(n-1,1);
    for k=1:n
%         p_footcontact(k,:) = mean(position(step(k,1):step(k,2),:));
%         p_footcontact(k,:) = mean(position(step(k,1)+10:step(k,2)-10,:));

%%% 之前是取减少前后20个数据点，现在改为减少前后10%。

    data_position = position(step(k,1):step(k,2),:);
       % 计算数据长度
    data_length = length(data_position);
    
    % 计算10%的位置 取整
    ten_percent_index = round(0.1 * data_length);
    
    % 取中间80%的数据
    middle_Data{k} = data_position(ten_percent_index +1 : data_length-ten_percent_index,:);

    p_footcontact(k,:) = mean(middle_Data{k});

    end
    for k=1:n-1
%         footsteplength(k) = norm(p_footcontact(k+1,:)-p_footcontact(k,:));
        footsteplength(k) = norm(p_footcontact(k+1,:)-p_footcontact(k,:));
        
        if footsteplength(k)<0.05 || footsteplength(k)>2
            footsteplength(k)=[];
        end
    end
    
end