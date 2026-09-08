function [stepsR] = fun_findSteps(contactMatrix)
%[stepsR, stepsL] = fun_findSteps(contactMatrix)
% takes in matrix with boolean for foot contact
% returns matrix with foot

% contactMatrix = contact;

% 脚触地时刻，导数�?1为脚抬起�?-1为脚落地；判断脚抬起时刻超前于脚落地时刻，目的计算双支撑相的时间长度
diffContact = diff(contactMatrix);

initContactR = [1;find(diffContact(:,1)==-1)];
lastContactR = [find(diffContact(:,1)==1); length(contactMatrix)];

if length(initContactR) ~= length(lastContactR)
    if length(initContactR) > length(lastContactR)
        initContactR = initContactR(1:end-1);
    else
        lastContactR = lastContactR(1:end-1);
    end
end

if all((lastContactR - initContactR)>0)
    stepsR = [initContactR, lastContactR];
else
    error('not handled')
end

end