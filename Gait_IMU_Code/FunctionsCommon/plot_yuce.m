%%% 绘图
%%
% 假设actualBBS和predictedBBS是您的数据
actualBBS = rand(100,1)*60; % 生成随机数据
predictedBBS = actualBBS + randn(100,1)*10; % 生成随机数据

% 计算预测值的标准误差
se = std(predictedBBS - actualBBS);

% 绘制实际值与预测值的散点图
figure;
subplot(2,1,1);
scatter(actualBBS, predictedBBS, 'b');
hold on;
plot([0,60], [0,60], 'r--'); % 原来的线
plot([0,60], [0,60]+se, 'r:'); % 添加的上方的线
plot([0,60], [0,60]-se, 'r:'); % 添加的下方的线
xlabel('Actual BBS');
ylabel('Predicted BBS');
title('Actual vs Predicted plot');

% 计算并绘制残差图
residuals = actualBBS - predictedBBS;
subplot(2,1,2);
scatter(predictedBBS, residuals, 'b');
hold on;
plot([0,80], [0,0], 'r--');
xlabel('Predicted BBS');
ylabel('Residual');
title('Residual plot');
