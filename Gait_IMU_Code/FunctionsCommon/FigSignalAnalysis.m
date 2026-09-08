function FigSignalAnalysis(dataAnalysis,fs)
t = 0:1/fs:(size(dataAnalysis, 2)-1)/fs; % 时间向量

% 存储名称
axisSet = ['X';'Y';'Z']';

% 绘制时域图
figure;
for i = 1:3
    subplot(3,3,i);
    plot(t, dataAnalysis(i, :));
    xlabel('时间 (秒)');
    ylabel('速度 (m/s)');
    title(['IMU','速度','的时域图 - 轴', axisSet(i)]);
end

% 进行快速傅里叶变换 (FFT)
for i = 1:3
    X = fft(dataAnalysis(i, :));
    N = length(X);
    f = (0:N-1)*(fs/N);
    subplot(3,3,i+3);
    plot(f, abs(X));
    xlabel('频率 (Hz)');
    ylabel('幅度');
    title(['IMU','速度','的频域图 - 轴', axisSet(i)]);
end

% 进行短时傅里叶变换 (STFT)
window = hamming(128); % 窗函数
noverlap = 64; % 重叠部分
nfft = 256; % FFT点数

for i = 1:3
    subplot(3,3,i+6);
    [S,F,T,P] = spectrogram(dataAnalysis(i, :), window, noverlap, nfft, fs);
    surf(T, F, 10*log10(P), 'EdgeColor', 'none');
    axis xy; axis tight; colormap(jet); view(0, 90);
    xlabel('时间 (秒)');
    ylabel('频率 (Hz)');
    title(['IMU','速度','的STFT时频谱图 - 轴', axisSet(i)]);
    colorbar;
end


% 进行小波变换
figure;
for i = 1:3
    subplot(3,1,i);
    [cfs, f] = cwt(dataAnalysis(i, :), 'amor', fs);
    surf(t, f, abs(cfs), 'EdgeColor', 'none');
    axis xy; axis tight; colormap(jet); view(0, 90);
    xlabel('时间 (秒)');
    ylabel('频率 (Hz)');
    title(['IMU','速度','的小波变换时频谱图 - ', axisSet(i)]);
    colorbar;
end
end