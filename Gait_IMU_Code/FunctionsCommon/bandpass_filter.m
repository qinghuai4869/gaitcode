
function filtered_data = bandpass_filter(data, fs, low_cutoff, high_cutoff, order)
    % 设计带通巴特沃斯滤波器
    [b, a] = butter(order, [low_cutoff high_cutoff] / (fs / 2), 'bandpass');

    % 对每一行数据进行滤波
    filtered_data = zeros(size(data));
    for i = 1:size(data, 1)
        filtered_data(i, :) = filtfilt(b, a, data(i, :));
    end

    % 显示原始数据和滤波后数据
    figure;
    subplot(2, 1, 1);
    plot(data');
    title('原始数据');
    legend('X','Y','Z')
    axis on
    grid on
    subplot(2, 1, 2);
    plot(filtered_data');
    title('滤波后数据');
    legend('X','Y','Z')
    axis on
    grid on
    % 计算并显示频率分布
    figure;
    for i = 1:size(data, 1)
        subplot(2, size(data, 1), i);
        [Pxx, F] = periodogram(data(i, :), [], [], fs);
        plot(F, 10*log10(Pxx));
        title(['原始数据频谱 - 通道 ', num2str(i)]);
        xlabel('频率 (Hz)');
        ylabel('功率谱密度 (dB/Hz)');

        subplot(2, size(data, 1), i + size(data, 1));
        [Pxx, F] = periodogram(filtered_data(i, :), [], [], fs);
        plot(F, 10*log10(Pxx));
        title(['滤波后数据频谱 - 通道 ', num2str(i)]);
        xlabel('频率 (Hz)');
        ylabel('功率谱密度 (dB/Hz)');
        axis on
        grid on
    end
end