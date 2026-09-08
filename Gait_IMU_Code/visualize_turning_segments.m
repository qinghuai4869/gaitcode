function visualize_turning_segments(trajectory, curvature_threshold, min_segment_length)
    % 检测拐弯段
    [segments, centers] = find_turning_segments(trajectory, curvature_threshold, min_segment_length);
    
    % 计算曲率用于绘图
    curvature = calculate_robust_curvature(trajectory, 3);
    
    % 创建图形窗口
    figure('Color', 'white', 'Position', [100 100 1200 600]);
    
    % 子图1：轨迹和拐弯段
    subplot(2,2,[1,3]);
    plot(trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 2, 'DisplayName', '轨迹');
    hold on;
    
    % 绘制每个拐弯段
    colors = lines(length(segments));
    for i = 1:length(segments)
        seg = segments{i};
        plot(seg(:,1), seg(:,2), 'o-', 'Color', colors(i,:), 'LineWidth', 3, ...
             'MarkerSize', 6, 'MarkerFaceColor', colors(i,:), ...
             'DisplayName', sprintf('拐弯段 %d', i));
        
        % 标记段中心点
        plot(centers(i,1), centers(i,2), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
    end
    
    grid on; axis equal;
    title(sprintf('轨迹拐弯段检测\n(曲率阈值: %.2f, 最小段长: %d)', curvature_threshold, min_segment_length));
    xlabel('X坐标'); ylabel('Y坐标');
    legend('Location', 'best');
    
    % 子图2：曲率变化
    subplot(2,2,2);
    plot(curvature, 'r-', 'LineWidth', 1.5);
    hold on;
    yline(curvature_threshold, 'k--', 'LineWidth', 1.5, 'Label', '曲率阈值');
    
    % 标记拐弯段范围
    for i = 1:length(segments)
        seg = segments{i};
        idx = find(ismember(trajectory, seg, 'rows'));
        xfill = [min(idx), max(idx), max(idx), min(idx)];
        yfill = [0, 0, max(curvature)*1.1, max(curvature)*1.1];
        fill(xfill, yfill, colors(i,:), 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    end
    
    grid on;
    title('曲率变化及拐弯段');
    xlabel('点索引'); ylabel('曲率');
    
    % 子图3：拐弯段统计
    subplot(2,2,4);
    if ~isempty(segments)
        seg_lengths = cellfun(@(x) size(x,1), segments);
        bar(seg_lengths, 'FaceColor', 'flat', 'EdgeColor', 'none');
        for i = 1:length(seg_lengths)
            text(i, seg_lengths(i), sprintf('%d点', seg_lengths(i)), ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
        end
        title('各拐弯段长度');
        xlabel('段编号'); ylabel('点数');
        grid on;
    end
    
    % 输出结果
    fprintf('检测到 %d 个拐弯段：\n', length(segments));
    for i = 1:length(segments)
        seg = segments{i};
        fprintf('段 %d: 起止点 [%d→%d], 中心点 (%.3f, %.3f), 长度 %d 点\n', ...
                i, find(ismember(trajectory, seg(1,:), 'rows')), ...
                find(ismember(trajectory, seg(end,:), 'rows')), ...
                centers(i,1), centers(i,2), size(seg,1));
    end
end