function [Rgs_init,YA_init]=initorient_wHeading(acc_s, headingEstimate)
% 初始方向估计
% Rgs=INITORIENT(acc_s, iflat)
%   Calculates an orientation using the assumptions:
%   - Subject is standing still
%   - x-axis of subject body (sensor frame) is in the XZ plane of the global reference
%     system.
%
%
% y_b   N*3 accelerometer output, expressed in the body coordinate frame. 
% ti    begin times & end times of intervals in which subject is standing
%       still. Each row contains 2 samplenumbers containing the start- and
%       end times of the interval.
% Rgs   [Xs Ys Zs]', where Xs, Ys, and Zs are the origin of global frame
%       expressed in the sensor frame
% Henk Luinge, 8-12-2001
% Martin Schepers, November 2005


    Nint = 1;
    Zs_acc = mean(acc_s,2);       % get mean of acc in flat interval
% Zs_acc=acc_s;
   
%%% Construct matri(x)(ces)   
% use y direction initially if x sensor is up...

    Xs=headingEstimate;%leg je vast
  
        Zs = Zs_acc./norm(Zs_acc);% meet je
        Ys = cross(Zs,Xs); % dus y bepalen
        Ys = Ys./norm(Ys);
        %if Zb(3)<0, Zb=-Zb;, end
        Xs = cross(Ys,Zs); % voor als z en x niet orthogonaal zijn
        Xs = Xs./norm(Xs);
        Rgs_init=[Xs Ys Zs]';
        YA_init=Zs;
    
end
