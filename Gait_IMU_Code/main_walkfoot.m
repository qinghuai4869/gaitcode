%%%%
% 步态应用论文
% FunctionslinearWalk 直线行走
% FunctionsFreedomWalk 曲线行走
% 已有存储数据格式不更改了，新采集得数据按照做（名字+1，2，3。。。）
%% 建立数据依次存储，顺序为名字，试次，
% 一开始运行首先完善导入路径，这里是批处理，建议用自己的数据改成单次导入修改：main_loadData存储路径即可
% 否则一运行就会报错。涉及隐私，不提供数据和数据路径。
%% 声明：请不要对代码中作者的文字部分进行网络评价，这些都是作者的思考，没有任何其他意思表达

clear 
clc
close all


prompt       = ['please choose model:'];
input_result = input(prompt,'s');
 
disp('Loads the algorithm settings and the IMU data')

addpath(genpath('FunctionsCommon'));

if input_result=='1' %2X6m
    Name_ = {'XXX'};%批处理，按照你的自己数据填写
    adjust_results = []; 
    ErrorInit_heading = [];
    ErrorInit_heading2 = [];
    Errorturns =[];
    addpath(genpath('FunctionsFreedomWalk'));
elseif input_result == '2'
    Name_ = {''};
    adjust_results = [];
    ErrorInit_heading = [];
    ErrorInit_heading2 = [];
    addpath(genpath('FunctionslinearWalkNew'));
else
    error('输入错误，请重新输入');
end

N = length(Name_(1,:));
filename = cell(1,N);
humanDataAVGRecord = zeros(N,16);
nonZeroColumns = zeros(N,1);
% adjust_results = [];
% adjust_limbLAAL= [];
% adjust_limbRAAL= [];
for j =2
[u,m_] = main_loadData(j,input_result);

global simdata;
SLpercentage = zeros(simdata.numFiles,2);
SL_diff = zeros(simdata.numFiles,2);         
StepLpercentage = zeros(simdata.numFiles,2);
StepL_diff = zeros(simdata.numFiles,2);
% write_sL = zeros(simdata.numFiles,16);
write_result = zeros(simdata.numFiles,16);
Xtrajectory_  = cell(1, simdata.numFiles);
LLL  = cell(5, simdata.numFiles);
RRR  = cell(5, simdata.numFiles);
RRstep  = cell(5, simdata.numFiles);
LLstep  = cell(5, simdata.numFiles);
LLvelocity = cell(3, simdata.numFiles);
RRvelocity = cell(3, simdata.numFiles);
LLcadence = cell(3, simdata.numFiles);
RRcadence = cell(3, simdata.numFiles);
LLstepWidth =cell(4,simdata.numFiles);
RRstepWidth =cell(4,simdata.numFiles);
LLstrideTime = cell(3,simdata.numFiles);
RRstrideTime = cell(3,simdata.numFiles);
LLStanceTime = cell(3,simdata.numFiles);
RRStanceTime = cell(3,simdata.numFiles);
LLSwingTime = cell(4,simdata.numFiles);
RRSwingTime = cell(3,simdata.numFiles);
number_empty =zeros(simdata.numFiles,2);
Lsum =zeros(simdata.numFiles,1);
Rsum =zeros(simdata.numFiles,1);
% adjust_results = [];
% adjust_limbLAAL= [];
% adjust_limbRAAL= [];
% ErrorInit_heading = [];
ErrorInit_heading0=[];
% write_process = zeros(simdata.numFiles,16);
%%
for  m = 18
%     m = 1:simdata.numFiles
    flag = 0;
%     delete bad data
    N_ = length(m_);
    for k= 1:N_
        if m==m_(k)
           flag = 1;
        end
    end

    if flag ==1
        continue;
    end

     if isempty(u{1,m})
        continue;
     end

    Rfoot0 = u{1,m}(1:6,:);
    Lfoot0 = u{2,m}(1:6,:); 
    diffL  = abs(length(Rfoot0)- length(Lfoot0));
    L      = min(length(Rfoot0), length(Lfoot0));

if input_result == '1'||input_result == '3'
     Rfoot0 = u{1,m}(1:6,150:L);
     Lfoot0 = u{2,m}(1:6,150:L);
elseif input_result == '2'
     Rfoot0 = u{1,m}(1:6,150:L);
     Lfoot0 = u{2,m}(1:6,150:L);
end


% 检查当前元素是否为空或剔除数据
    if isempty(Rfoot0) || isempty(Lfoot0)  || diffL > 100 
%         number_empty = [number_empty;m];
         continue; % 跳过当前迭代
    end
    

if input_result=='1'|| input_result == '2'|| input_result == '3'
    i = 0;
   while 1
        main_trajectory;

%         本次试验每次循环存放航向角误差
        ErrorInit_heading = [ErrorInit_heading; m i simdata.init_heading1 ...
            simdata.init_heading2 head_angle(3) head_angle(6)];
        i = i+1;
        number_empty(m,2) = i;

        if i==1||i==2
            ErrorInit_heading0 = [ErrorInit_heading0 i-1 simdata.init_heading1 ...
            simdata.init_heading2 head_angle(3) head_angle(6)];
        end

        if (abs(head_angle(3)) < 0.005 && abs(head_angle(6))<0.005 )|| i > 5
            ErrorInit_heading0 = [ErrorInit_heading0 i-1 simdata.init_heading1 ...
            simdata.init_heading2 head_angle(3) head_angle(6)];
            if i==1
               ErrorInit_heading0 = [ErrorInit_heading0 i-1 simdata.init_heading1 ...
               simdata.init_heading2 head_angle(3) head_angle(6)];
            end
            break;
        end

        simdata.init_heading1 = simdata.init_heading1 + head_angle(3);
        simdata.init_heading2 = simdata.init_heading2 + head_angle(6);
%         simdata.range_constraint = mean(strideLengthIMU_AVG)*1.5;
   end

   if flag ==0
   %不同次试验保存航向角
       ErrorInit_heading1 = [m ErrorInit_heading0];
       ErrorInit_heading2 = [ErrorInit_heading2;ErrorInit_heading1];
       clear ErrorInit_heading1
   end
       ErrorInit_heading0 =[];      

% %   这里加一个跳出循环的结果，单写    
    simdata.init_heading1 = 0;
    simdata.init_heading2 = 0;
%     simdata.range_constraint=1.0;
end


    if flag ==1
        continue;
    end

%     simdata.init_heading1 = 0;
%     simdata.init_heading2 = 0; 
%%
% 2X6m的误差
if input_result=='1'
    Errorturns0 = [m,Errorturn]; 
    Errorturns = [Errorturns;m,Errorturn];
    clear Errorturns0
end
% %%   adjust_results：健康人是左边在左边，患者的步长和步宽是反的。%
% %    因对齐问题，患者个数不好弄，数不齐
%     %%%%% 按照试次排列结果 健康人
if input_result == '2'
% 直线定长行走
    adjust_results0(1,:) = [strideLength_L_IMU*100 strideLength_R_IMU*100];
    adjust_results0(2,:) = [stepLength_L_IMU *100 stepLength_R_IMU *100];
    adjust_results0(3,:) = [LeftIMUstrideVelocity RightIMUstrideVelocity];
    adjust_results0(4,:) = [LeftIMUCadence RightIMUCadence];
    adjust_results0(5,:) = [LStrideTimeIMU RStrideTimeIMU];
    adjust_results0(6,:) = [LStanceTimeIMU RStanceTimeIMU];
    adjust_results0(7,:) = [LSwingTimeIMU RSwingTimeIMU];
    adjust_results0(8,:) = [m * ones(1,length(strideLength_L_IMU)),zeros(1,length(strideLength_R_IMU))];
    adjust_results = [adjust_results,adjust_results0];
    clear adjust_results0;
end

end

end % 所有人数据
%% 保存数据
if input_result == '2'
Number = adjust_results(8,:)';
strideLength = adjust_results(1,:)';
stepLength = adjust_results(2,:)';
strideVelocity = adjust_results(3,:)';
Cadence = adjust_results(4,:)';
StrideTime = adjust_results(5,:)';

savefilename =['XXX',...
 Name_{j}, '位置6_1.5-60.xls'];
sheet1 = ['60cm步态参数'];
T_gaitparameters = table(Number,strideLength,stepLength,Cadence,StrideTime);
writetable(T_gaitparameters,savefilename,'Sheet',sheet1,'Range','D1');
end

if input_result== '1'
    Number1    = Errorturns(:,1);
    errorTurn1 = Errorturns(:,2);
    errorTurn2 = Errorturns(:,3);
    errorTurn3 = Errorturns(:,4);
    errorend   = Errorturns(:,5);
    T_errorTurns = table(Number1,errorTurn1,errorTurn2,errorTurn3,errorend);
    savefilename =[''];% 填写存储路径
    sheet2 = ['拐弯误差+重点误差'];
    writetable(T_errorTurns,savefilename,'Sheet',sheet2,'Range','D1');
end

%% save data to excel Every body
% saveDataToExcel;
% clear adjust_results;

% LLL11=LLL(1,6)
% writecell(LLL',filename,'Sheet',2);
% %,'Range',A1
% writecell(RRR',filename,'Sheet',2,'Range','AA1');
% % ,'WriteMode','append'
% writecell(LLstep',filename,'Sheet',2,'Range','BA1');
% writecell(RRstep',filename,'Sheet',2,'Range','CA1');
 %%
 disp('every subject data finish')
 sprintf(Name_{j})
 
% end % 单人数据
%%
 
disp('ALL Data Save finish')

