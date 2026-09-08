% 设计一个截止频率为 10 的 2 阶低通巴特沃斯滤波器
fc = 10;fs = 100;
[b,a] = butter(2,fc/(fs/2));

% 计算并绘制滤波器的频率响应
figure;
freqz(b, a);

% 生成一个包含两个正弦波的信号
Fs = 100; % 采样频率
t = 0:1/Fs:1; % 时间序列
x = Lfoot0'; % 包含两个正弦波的信号
% x2 = Rfoot0'; % 包含两个正弦波的信号

% 计算并绘制信号的频谱
figure;
N = length(x);
f = (0:N-1)*(Fs/N); % 频率序列
X = fft(x)/N; % 计算信号的频谱
plot(f, abs(X));
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('Original Signal Spectrum');

% 对信号进行滤波
y = filtfilt(b, a, x);

% 计算并绘制滤波后信号的频谱
figure;
Y = fft(y)/N; % 计算滤波后信号的频谱
plot(f, abs(Y));
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('Filtered Signal Spectrum');
