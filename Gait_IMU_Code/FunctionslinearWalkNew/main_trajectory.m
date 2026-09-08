%% 获取数据
% clear all
% close all
% addpath(genpath('Functions'))
% disp('Loads the algorithm settings and the IMU data')
% [u,strideLengthTruthValue] = main_loadData();
% Rfoot0 = u{1,2}(1:6,1:1300);
% Lfoot0 = u{2,2}(1:6,1:1300);
% 
% strideLengthTruthValue = strideLengthTruthValue{2};
%%% 考虑去掉静止噪声？
%% 由于肌肉伪影，滤波
%%%% 去掉高频噪声 2阶截止频率为10适合穿鞋
% fc = 20;
% fs = 100;
% [b,a] = butter(2,fc/(fs/2));
% % filter_com_a = filter(b,a,prior_com_a');
% Rfoot = filtfilt(b,a,Rfoot0')';
% Lfoot = filtfilt(b,a,Lfoot0')';
Rfoot = Rfoot0;
Lfoot = Lfoot0;

%%
%         r_w_a = 0.85;
%         r_w_w = 0.8;
%         l_w_a = 0.85;
%         l_w_w = 0.8;

        r_w_a = 0.8;
        r_w_w = 0.75;
        l_w_a = 0.8;
        l_w_w = 0.75;  
%% zupt 零速率更新
[zupt_r,Rt_stride,Rt_swing,Rt_stance,Rpos_hr,Rpos_fc,Rpos_point] = main_segmentation(Rfoot(1:3,:),Rfoot(4:6,:),r_w_a,r_w_w);
[zupt_l,Lt_stride,Lt_swing,Lt_stance,Lpos_hr,Lpos_fc,Lpos_point] = main_segmentation(Lfoot(1:3,:),Lfoot(4:6,:),l_w_a,l_w_w);

% [zupt_r,] = main_segmentation(Rfoot(1:3,:),Rfoot(4:6,:),r_w_a,r_w_w);
% [zupt_l] = main_segmentation(Lfoot(1:3,:),Lfoot(4:6,:),l_w_a,l_w_w);

% %% imu和足底压力判断先行脚是否一值,一致继续，不一致跳出当前迭代
% if (Rpos_hr(1) < Lpos_hr(1)  && TruthValue_row(1)==1)||(Rpos_hr(1) > Lpos_hr(1)  && TruthValue_row(1)==2) || Rpos_hr(1) == Lpos_hr(1) 
%     flag =1;
% end


%% 时间tk
N= length(Rfoot);
time = zeros(1,N);
Ts = 1/100;
% time = Ts:Ts:N*Ts;
time = 0:Ts:(N-1)*Ts;

% figure
% clf
% subplot(2,1,1)
% plot(time',Rfoot(1:3,:)','LineWidth', 1);

% % 设置字体和大小 
% % set(gca,'FontName','Times New Roman','FontSize',12)
% set(gca,'FontSize',10);
% 
% % 设置横坐标和纵坐标的范围
% xlim([0,12]);
% ylim([-40,40]);
% % 设置坐标轴的刻度，每隔 2/10 递增
% xticks(0:4:12);
% yticks(-40:20:40);
% 
% 
% xlabel('时间(s)');
% ylabel('右脚加速度');
% 
% 
% subplot(2,1,2)
% plot(time',Lfoot(1:3,:)','LineWidth', 1);
% 
% % 设置字体和大小 
% % set(gca,'FontName','Times New Roman','FontSize',12)
% set(gca,'FontSize',10);
% 
% % 设置横坐标和纵坐标的范围
% xlim([0,12]);
% ylim([-40,40]);
% % 设置坐标轴的刻度，每隔 2/10 递增
% xticks(0:4:12);
% yticks(-40:20:40);
% 
% 
% xlabel('时间(s)');
% ylabel('左脚加速度');
% 
% % f = gcf;
% % f.PaperUnits = 'centimeters'; % 设置单位为厘米
% % f.PaperPosition = [0 0 1 1]; % 设置图片的宽度为16厘米，高度为24厘米
% % print(f, '患者4-13加速度.png', '-dpng', '-r300'); % 保存为PNG格式，分辨率为300
% % saveas(f, '患者4-13加速度.fig');
% 
% figure
% clf
% subplot(2,1,1)
% plot(time',Rfoot(4:6,:)','LineWidth', 1);
% % 设置字体和大小 
% % set(gca,'FontName','Times New Roman','FontSize',12)
% set(gca,'FontSize',10);
% 
% % 设置横坐标和纵坐标的范围
% xlim([0,12]);
% ylim([-4,8]);
% % 设置坐标轴的刻度，每隔 2/10 递增
% xticks(0:4:12);
% yticks(-4:2:8);


% xlabel('时间(s)');
% ylabel('右脚角速度');
% subplot(2,1,2)
% plot(time',Lfoot(4:6,:)','LineWidth', 1);
% % 设置字体和大小 
% % set(gca,'FontName','Times New Roman','FontSize',12)
% set(gca,'FontSize',10);
% 
% % 设置横坐标和纵坐标的范围
% xlim([0,12]);
% ylim([-4,8]);
% % 设置坐标轴的刻度，每隔 2/10 递增
% xticks(0:4:12);
% yticks(-4:2:4);
% 
% 
% xlabel('时间(s)');
% ylabel('左脚角速度');
% % 图片尺寸 6*10
% figure
% clf
% plot(time',zupt_r,'k');
% hold on
% plot(time',zupt_l,'r');
% title('zupt');
% legend('右','左');



%% Run the Kalman filter
contact = [zupt_r;zupt_l];
u1 = [Rfoot;Lfoot];
[x_h,cov,a_new,vel_error_r,vel_error_l,d_h]=ZUPTaidedINS(u1,contact);
%%%%% 速度漂移
% [x_h,cov,a] = ZUPTaidedINS_re(u1,contact,vel_error_r,vel_error_l);
%%% x-轴并不完全指向前进方向，和想要的方向误差怎么搞？
%%% 采用脚结束的最后静止的方向和脚初始方位，作为？但是两只脚走完也不是平行的哦
%%%% 怎么让另一只和他对齐呢？ 找找办法！采用算法对齐好了。
%% 分割每步
stepsR = fun_findSteps(zupt_r');
stepsL = fun_findSteps(zupt_l');

%% 
% 加这里，反馈回去

%% View the result 
% disp('Views the data')
% view_data;

%% 时间：stride Time ,stance time ,swing time
% 所有均统一为按照步幅个数来算，不再做二次处理，但是有可能，下面的个数不足？

LStrideTimeIMU = Lt_stride;
RStrideTimeIMU = Rt_stride; 


RSwingTimeIMU = Rt_swing;
LSwingTimeIMU = Lt_swing;

RStanceTimeIMU = Rt_stance;
LStanceTimeIMU = Lt_stance;



%% 步幅
% load("contanct1.mat");

Rstride = stridelength(x_h(1:2,:)',stepsR);
Lstride = stridelength(x_h(10:11,:)',stepsL);

Rstride = Rstride';
Lstride = Lstride';
timeJudge = mean(stepsR(1:3,2) - stepsL(1:3,2));
if timeJudge > 0
% 先走左脚,这里如果先走右脚，需要改变。
    strideLength_L_IMU = Lstride(2:end);
    strideLength_R_IMU = Rstride(1:end-1);
else
    % 先走右脚
    strideLength_L_IMU = Lstride(1:end-1);
    strideLength_R_IMU = Rstride(2:end);
end

strideLengthIMU_AVG = mean([abs(strideLength_L_IMU) abs(strideLength_R_IMU)]); %% 单位统一

%% 步长计算和步宽计算:这里多余了，应当在里面处理掉，
%  左右脚已经判断过，当时应该是考虑到了需要全部数据把，输出两个就好了，额么么么
[RstepLength,LstepLength,RstepWidth,LstepWidth] = ...
    fun_stepsLength2(x_h(1:2,:)',x_h(10:11,:)',stepsR,stepsL);

% timeJudge = mean(stepsR(1:3,2) - stepsL(1:3,2));
if timeJudge > 0
% 先走左脚
stepLength_L_IMU = LstepLength(2:end);
stepLength_R_IMU = RstepLength(1:end);
else
    % 先走右脚
stepLength_L_IMU = LstepLength(1:end);
stepLength_R_IMU = RstepLength(2:end);
end

%% 步速以及步频

RightIMUstrideVelocity = Rstride(1:end-1)./Rt_stride;
LeftIMUstrideVelocity = Lstride(2:end)./Lt_stride;

RightIMUCadence  = 2./Rt_stride.*60;
LeftIMUCadence  = 2./Lt_stride.*60;


%% T_rest 在foot_flat
%%%%%%
% Rpos_ff = [1,Rpos_fc;Rpos_hr,length(Rfoot)]; 
% Rpos_rest = fix(mean(Rpos_ff));
% Lpos_ff = [1,Lpos_fc;Lpos_hr,length(Lfoot)]; 
% Lpos_rest = fix(mean(Lpos_ff));
% 
% [R_v,R_p] = main_velocity(Racc_Rotate,Rpos_rest);
% [L_v,L_p] = main_velocity(Lacc_Rotate,Lpos_rest);
% 
% N = length(R_v);
% for k = 1:N
%     L0 = R_p{k};
%     L1(:,k)= L0(1:2,end);
%     RL(:,k) = norm(L1(:,k));
%     clear L0
% end
% 
% for k = 1:N
%     L0 = L_p{k};
%     L2(:,k)= L0(1:2,end);
%     LL(:,k) = norm(L2(:,k));
%     clear L0
% end
%

%% 初始航向纠正  初始位置！
% Rhead = mean(x_h(1:3,end-30:end-1),2)
Rhead = mean(x_h(1:3,end-30:end-1),2) - mean(x_h(1:3,1:30),2);%% 维度不对
Rseg_r = initorient_wHeading(Rfoot(1:3,10:40),Rhead);
% Lhead = mean(x_h(10:12,end-30:end-1),2);%% 维度不对 Lhead = mean(x_h(10:12,end-30:end-1),2);
Lhead = mean(x_h(10:12,end-30:end-1),2) - mean(x_h(10:12,1:30),2);
Lseg_r = initorient_wHeading(Lfoot(1:3,10:40),Lhead);

head_angle=[];
head_angle(1)=atan2(Rseg_r(3,2),Rseg_r(3,3));

% pitch
head_angle(2)=-atan(Rseg_r(3,1)/sqrt(1-Rseg_r(3,1)^2));

%yaw
head_angle(3)=atan2(Rseg_r(2,1),Rseg_r(1,1));

head_angle(4)=atan2(Lseg_r(3,2),Lseg_r(3,3));

% pitch
head_angle(5)=-atan(Lseg_r(3,1)/sqrt(1-Lseg_r(3,1)^2));

%yaw
head_angle(6)=atan2(Lseg_r(2,1),Lseg_r(1,1));





%% 速度和轨迹图
% figure
% 速度
% clf
% subplot(2,1,1);
% plot(time,x_h(4:6,:)','LineWidth', 1)
% xlabel('时间 (s)')
% ylabel('右脚速度 (m/s)')
% set(gca,'FontSize',10);
% axis([0,12,-1,2]);
% hold on
% subplot(2,1,2)
% plot(time,x_h(13:15,:)','LineWidth', 1)
% xlabel('时间 (s)')
% ylabel('左脚速度 (m/s)')
% set(gca,'FontSize',10);
% axis([0,12,-1,2]);
% % % 
% figure
% %位置
% clf
% plot(x_h(1,:),x_h(2,:),'b','LineWidth', 1)
% hold on
% plot(x_h(10,:),x_h(11,:),'r','LineWidth', 1)
% hold on
% plot(x_h(1,1),x_h(2,1),'gs','LineWidth', 1)
% title('Trajectory')
% legend('Right Foot Trajectory','Left Foot Trajectory','Start point')
% legend('右脚轨迹','左脚轨迹')
% xlabel('x (m)')
% ylabel('y (m)')
% axis([0,6,-1,1]);
% set(gca,'FontSize',10);
% axis equal
% box on
  
%% trajectory
sprintf('Horizontal error = %0.5g , Spherical error = %0.5g',sqrt(sum((x_h(1:2,end)).^2)), sqrt(sum((x_h(1:3,end)).^2)));

% sprintf('Magnetic Heading Update = %s , Roll Pitch Update from Accelerometer = %s',simdata.mhupt, simdata.angleupdatefromaccel)
