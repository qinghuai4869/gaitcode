function [rtk,time_stride,time_swing,time_stance,pos_hr,pos_fc,pos_timeGaitPhaseDuration]=main_segmentation(f_imu,omega_imu,w_a,w_w)

%% 时间tk
N= length(f_imu);
Ts = 1/100;
% time = zeros(1,N);
time = 0:Ts:(N-1)*Ts;

%% anto threshold adataption
% w_a = 0.85;%%%% 由于光脚，肌肉伪影！
% w_w = 0.8;

% w_a = 0.8;%%%% 由于光脚，肌肉伪影！
% w_w = 0.75; %%%% 相比角速度变化，角速度干扰更大！！

% w_a = 0.9;%%%% 过度使用零速率，导致左脚计算偏小
% w_w = 0.85;

%范数
    N= length(f_imu);
    norm_acc = zeros(1,N);
    for k=1:N
        norm_acc(k) = norm(f_imu(:,k));
    %     norm_acc(k) = norm(omega_imu(:,k));
    end
    norm_gyr = zeros(1,N);
    for k=1:N
        norm_gyr(k) = norm(omega_imu(:,k));
    %     norm_acc(k) = norm(omega_imu(:,k));
    end
%     2023.9.13更改9.7608为9.7860
%     norm_reacc = norm_acc - 9.7608;
    norm_reacc = norm_acc - 9.7926;
    norm_reacc = abs(norm_reacc);   
    [acc_th_final] = fun_autoThreshold(time,N,w_a,norm_reacc);   %加速度阈值
    [agr_th_final] = fun_autoThreshold(time,N,w_w,norm_gyr);      %角速度阈值
% 
% figure
% clf
% subplot(2,1,1)
% plot(time',norm_reacc','b');title('加速度范数');
% hold on
% a1=[0.01 time(end)];
% b1=[acc_th_final acc_th_final];
% plot(a1,b1,'r','LineWidth',1.5);
% xlabel('time');ylabel('accelerometer norm a(tk)');
% legend('加速度范数减去重力加速度','减去重力加速度的阈值');
% subplot(2,1,2)
% plot(time',norm_gyr','b');title('角速度范数');
% hold on
% a2=[0.01 time(end)];
% b2=[agr_th_final agr_th_final];
% plot(a2,b2,'r','LineWidth',1.5);
% xlabel('time');ylabel('gyrscope norm w(tk)');
% legend('角速度范数','角速度范数的阈值');
%% acceleration_based rest signal ra(tk)
%  完全落地到平坦时间
% pre_atk = norm_acc - 9.7608;
pre_atk = norm_acc - 9.7926;
pre_atk = abs(pre_atk);  %%% 前后应该用一个啊，怎么会分开写呃呃呃，2023.9.13更改
pre_wtk = norm_gyr;
h_a = 0.23;
h_w = 0.23;

ra_finial= fun_accelerationBasedRestSignal(pre_atk,h_a,N,acc_th_final);
rw_finial= fun_accelerationBasedRestSignal(pre_wtk,h_w,N,agr_th_final);

%% 完全落地到平坦时间融合
rtk_combined = ra_finial + rw_finial;
rtk_pos = rtk_combined>1;
rtk_combined(rtk_pos)=1;
rtk = fun_peliminatePhases(rtk_combined);

%%%% time : heel rise and full contact
pos_hr =find(diff(rtk)==1);
pos_fc = find(diff(rtk)==-1);  %%% 这个是1还是0
% time_hr = time(pos_hr);
% time_fc = time(pos_fc) + 0.01;
% y_hr = zeros(1,length(time_hr));
% y_hr_t = y_hr - 0.02;
% y_fc = zeros(1,length(time_fc));
% y_fc_t = y_fc - 0.02;

%%
time_hr = time(pos_hr);
time_fc = time(pos_fc) + 0.01;
y_hr = zeros(1,length(time_hr));
y_hr_t = y_hr - 0.02;
y_fc = zeros(1,length(time_fc));
y_fc_t = y_fc - 0.02;

%%%%%%%%% 之前是先取阈值范围后取点，纠正点后，改rtk %%%%%%%%
rtk_pos = [1:pos_fc,pos_hr:length(rtk)];
%% 
% figure
% clf
% plot(time',rtk','k','LineWidth',1);title('rest:r(tk)');
% xlabel('time');ylabel('r(tk)');
% hold on
% scatter(time_hr,y_hr,60,'filled','b','marker','o','MarkerEdgeColor','b');
% % scatter(time_hr,y_hr,60,'filled','d','marker','square');
% text(time_hr,y_hr_t,'t,hr');
% hold on
% scatter(time_fc,y_fc,60,'filled','r','marker','o','MarkerEdgeColor','r');
% text(time_fc,y_fc_t,'t,fc');


%% 
%  %%%% 纠正foot flat阈值
[pos_hr,pos_fc,rtk] = fun_reFootFlat(pos_hr,pos_fc,acc_th_final,agr_th_final,norm_reacc,norm_gyr,rtk); 

[pos_hr,pos_fc,rtk] = fun_FootPP(pos_hr,pos_fc,acc_th_final,agr_th_final,norm_reacc,norm_gyr,rtk); 

%%
time_hr = time(pos_hr);
time_fc = time(pos_fc) + 0.01;
y_hr = zeros(1,length(time_hr));
y_hr_t = y_hr - 0.02;
y_fc = zeros(1,length(time_fc));
y_fc_t = y_fc - 0.02;

%%%%%%%%% 之前是先取阈值范围后取点，纠正点后，改rtk %%%%%%%%
rtk_pos = [1:pos_fc,pos_hr:length(rtk)];
%% 
% figure
% clf
% plot(time',rtk','k','LineWidth',1);title('rest:r(tk)');
% xlabel('time');ylabel('r(tk)');
% hold on
% scatter(time_hr,y_hr,60,'filled','b','marker','o','MarkerEdgeColor','b');
% % scatter(time_hr,y_hr,60,'filled','d','marker','square');
% text(time_hr,y_hr_t,'t,hr');
% hold on
% scatter(time_fc,y_fc,60,'filled','r','marker','o','MarkerEdgeColor','r');
% text(time_fc,y_fc_t,'t,fc');

%减去重力加速度范数+角速度范数+rtk
% figure
% clf
% p1=plot(time',norm_reacc,'LineWidth',1,'Color',[0.30,0.75,0.93]);
% 
% hold on
% p2=plot(time',norm_gyr,'LineWidth',1,'Color',[1,0.65,0]);
% hold on
% p3=plot(time',5*rtk','r','LineWidth',1);%title('rest:r(tk)');
% hold on
% scatter(time_hr,y_hr,60,'filled','y','marker','o','MarkerEdgeColor','y');
% % scatter(time_hr,y_hr,60,'filled','d','marker','square');
% text(time_hr,y_hr_t,'t,hr');
% hold on
% scatter(time_fc,y_fc,60,'filled','y','marker','o','MarkerEdgeColor','y');
% text(time_fc,y_fc_t,'t,fc');
% lgd=legend('The linear acceleraction','The angular velocity ','Gait cycle detection');
% lgd.FontSize=10;
% lgd.Orientation = 'horizon';
% lgd.Location = 'northeast';
% lgd.FontName= 'Times New Roman';
% axis([0,10,-2,30]);
% xlabel('Time (s)','FontName','Times New Roman','FontSize',14);
% ylabel('Norm','FontName','Times New Roman','FontSize',14);
% set(gca, 'Fontname', 'Times New Roman','FontSize',14,'LineWidth',1);
% set(gca,'looseInset',[0 0 0 0]);
% print(gcf, '-dtiff', '-r600', './阈值13-0.tif')


%%  脚趾离地时刻
 [pos_to,pos_TorelativetoHR] = fun_timeToeOff(pos_hr,pos_fc,omega_imu);

%% 初始接触检测
 [pos_ic,pos_ICrelativetoFC] = fun_timeInitContact(f_imu,pos_to,pos_fc);

%% 步幅stride和步态周期和步频
pos_timeGaitPhaseDuration= [pos_hr;pos_to;pos_ic;pos_fc];
% pos_timeGaitPhaseDuration= [[1,pos_ic];[1 pos_fc];[pos_hr 0];[pos_to 0]];

time_stride = Ts * diff(pos_ic);
M = length(time_stride);
time_swing = zeros(1,M);
for k =1:M
    time_swing(k) = Ts * (pos_ic(k+1) - pos_to(k+1)); 
end
time_stance = time_stride - time_swing;

percentage_swing = time_swing./time_stride;
percentage_stance = time_stance./time_stride;

percentage_swingToStance = time_swing./time_stance;

avg_swing = mean(time_swing);
avg_stance = mean(time_stance);

cadence = 2./time_stride;
%% 步态周期图
%%%%% 重新将划分周期相下平移 
%%%%于加速度范数减去重力和角速度范数相比
y_hr1 = y_hr - 10;
y_fc1 = y_fc -10;
y_hr_t1 =y_hr_t -10.48;
y_fc_t1 = y_fc_t - 10.48;
rtk1 = rtk -10;
%%%%%% 初始接触和脚趾离地
time_ic = time(pos_ic);
time_to = time(pos_to);
y_ic = y_hr;
y_ic1 = y_hr1;
y_to1 = y_fc1;
y_ic_t1 =y_hr_t -10.48;
y_to_t1 = y_fc_t - 10.48;

prior_com_a = [f_imu;omega_imu];

% figure   %原始数据加速度+角速度 周期图
% clf
% plot(time',prior_com_a');title('传感器+步态事件周期');
% hold on
% plot(time',rtk1','r','LineWidth',1);title('rest:r(tk)');
% hold on
% scatter(time_hr,y_hr1,40,'filled','k','marker','o','MarkerEdgeColor','y');
% % scatter(time_hr,y_hr,60,'filled','d','marker','square');
% text(time_hr,y_hr_t1,'t,hr');
% hold on
% scatter(time_fc,y_fc1,40,'filled','k','marker','o','MarkerEdgeColor','y');
% text(time_fc,y_fc_t1,'t,fc');
% legend('a(x)','a(y)','a(z)','w(x)','w(y)','w(z)','步态时间周期');
% hold on
% scatter(time_ic,y_ic1,40,'filled','b','marker','o','MarkerEdgeColor','y');
% scatter(time_ic,y_ic,60,'filled','d','marker','square');
% text(time_ic,y_ic_t1,'t,ic');
% hold on
% scatter(time_to,y_to1,40,'filled','b','marker','o','MarkerEdgeColor','y');
% text(time_to,y_to_t1,'t,to');

end 
