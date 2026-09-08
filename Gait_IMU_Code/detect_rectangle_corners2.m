function detect_rectangle_corners(traj,t)
    rng(0); % 固定随机种子，确保结果可复现
    
    % 1. 生成更陡峭的转弯轨迹
    x=traj(1,:);
    y=traj(2,:);
        
    % 2. 检测拐弯区间（而非单个点）
    [turn_segments, angles_deg, t_angles] = detect_turn_segments(x, y, t);
    
    % 3. 可视化轨迹和拐弯区间
    visualize_turn_segments(x, y, t, turn_segments, angles_deg, t_angles);
end

function [turn_segments, angles_deg, t_angles] = detect_turn_segments(x, y, t)
    % 检测拐弯区间（连续的拐弯过程）
    % 输出：turn_segments为4×2矩阵，每行是[start_idx, end_idx]（拐弯区间的起止索引）
    
    % 1. 轻度平滑（保留拐弯细节）
    window = 5;
    x_smooth = movmean(x, window);
    y_smooth = movmean(y, window);
    
    % 2. 计算方向角变化
    dx = diff(x_smooth);
    dy = diff(y_smooth);
    dir_angle = atan2(dy, dx);  % 方向角（弧度）
    d_angle = diff(dir_angle);  % 方向角变化
    d_angle = mod(d_angle + pi, 2*pi) - pi;  % 标准化到[-π, π]
    angle_change = abs(d_angle);  % 拐弯角度大小
    angles_deg = rad2deg(angle_change);  % 转换为角度
    t_angles = t(2:end-1);  % 角度对应的时间点
    
    % 3. 识别拐弯区间：连续的角度变化较大的区域（60~120度）
    is_turn_point = angles_deg >= 60 & angles_deg <= 120;
    num_points = length(is_turn_point);
    
    % 提取连续的拐弯区间（至少包含5个点，避免噪声）
    turn_candidates = [];
    current_start = [];
    for i = 1:num_points
        if is_turn_point(i)
            if isempty(current_start)
                current_start = i;  % 开始新的拐弯区间
            end
        else
            if ~isempty(current_start)
                current_end = i-1;
                % 只保留长度≥5的区间（确保是有效拐弯过程）
                if current_end - current_start + 1 >= 5
                    turn_candidates = [turn_candidates; current_start, current_end];
                end
                current_start = [];
            end
        end
    end
    % 处理最后一个区间
    if ~isempty(current_start)
        current_end = num_points;
        if current_end - current_start + 1 >= 5
            turn_candidates = [turn_candidates; current_start, current_end];
        end
    end
    
    % 4. 用几何约束筛选4个拐弯区间（对应矩形的4个拐角区域）
    L = 6; W = 2;
    target_areas = [  % [xmin, xmax, ymin, ymax]
        L-1.5, L+1.5, 0-1.5, 0+1.5;  % 右下拐角区域
        L-1.5, L+1.5, W-1.5, W+1.5;  % 右上拐角区域
        0-1.5, 0+1.5, W-1.5, W+1.5;  % 左上拐角区域
        0-1.5, 0+1.5, 0-1.5, 0+1.5   % 左下拐角区域
    ];
    
    turn_segments = [];
    for i = 1:4  % 为每个目标区域匹配一个拐弯区间
        area = target_areas(i,:);
        xmin = area(1); xmax = area(2);
        ymin = area(3); ymax = area(4);
        
        % 计算每个候选区间与目标区域的匹配度（区间中点是否在区域内）
        best_segment = [];
        best_score = Inf;
        for j = 1:size(turn_candidates,1)
            seg = turn_candidates(j,:);
            mid_idx = round(mean(seg)) + 1;  % 区间中点（映射回原始轨迹索引）
            x_mid = x_smooth(mid_idx);
            y_mid = y_smooth(mid_idx);
            
            % 计算中点到区域中心的距离（距离越小越匹配）
            area_center = [(xmin+xmax)/2, (ymin+ymax)/2];
            dist = sqrt((x_mid - area_center(1))^2 + (y_mid - area_center(2))^2);
            
            % 优先选择距离近且角度变化接近90度的区间
            seg_angles = angles_deg(seg(1):seg(2));
            angle_score = mean(abs(seg_angles - 90));  % 角度平均偏差
            total_score = dist + 0.1*angle_score;  % 综合评分
            
            if total_score < best_score
                best_score = total_score;
                best_segment = seg;
            end
        end
        
        % 如果没有候选区间，手动创建一个（极端情况）
        if isempty(best_segment)
            % 在目标区域内找连续点作为拐弯区间
            in_area = (x_smooth >= xmin) & (x_smooth <= xmax) & ...
                      (y_smooth >= ymin) & (y_smooth <= ymax);
            area_idx = find(in_area);
            if length(area_idx) >= 5
                best_segment = [area_idx(1), area_idx(min(5, length(area_idx)))];
            else
                best_segment = [area_idx(1), area_idx(end)];  % 最短区间
            end
            best_segment = best_segment - 1;  % 映射回角度索引
        end
        
        % 保存拐弯区间（转换为原始轨迹索引）
        turn_segments = [turn_segments; best_segment(1)+1, best_segment(2)+1];
    end
    
    % 5. 按时间排序拐弯区间
    [~, sort_idx] = sort(mean(turn_segments,2));
    turn_segments = turn_segments(sort_idx,:);
end

function visualize_turn_segments(x, y, t, turn_segments, angles_deg, t_angles)
    % 可视化轨迹和拐弯区间
    figure('Position', [100, 100, 1200, 800]);
    
    % 1. 轨迹图（高亮显示拐弯区间）
    subplot(2,1,1);
    hold on; axis equal; grid on;
    
    % 绘制整体轨迹
    plot(x, y, 'b-', 'LineWidth', 1, 'DisplayName', '直线段');
    
    % 高亮显示每个拐弯区间
    colors = lines(4);  % 4种颜色区分不同拐弯
    for i = 1:4
        seg = turn_segments(i,:);
        plot(x(seg(1):seg(2)), y(seg(1):seg(2)), 'LineWidth', 3, ...
            'Color', colors(i,:), 'DisplayName', ['拐弯区间 ', num2str(i)]);
        % 标记区间的起止点
        plot(x(seg(1)), y(seg(1)), 'ko', 'MarkerSize', 6, 'LineWidth', 1.5);
        plot(x(seg(2)), y(seg(2)), 'ks', 'MarkerSize', 6, 'LineWidth', 1.5);
    end
    
    % 标记起点和终点
    plot(x(1), y(1), 'go', 'MarkerSize', 8, 'DisplayName', '起点');
    plot(x(end), y(end), 'mo', 'MarkerSize', 8, 'DisplayName', '终点');
    
    % 绘制目标区域（辅助参考）
    L = 6; W = 2;
    rectangle('Position', [L-1.5, 0-1.5, 3, 3], 'EdgeColor', 'k', 'LineStyle', ':');
    rectangle('Position', [L-1.5, W-1.5, 3, 3], 'EdgeColor', 'k', 'LineStyle', ':');
    rectangle('Position', [0-1.5, W-1.5, 3, 3], 'EdgeColor', 'k', 'LineStyle', ':');
    rectangle('Position', [0-1.5, 0-1.5, 3, 3], 'EdgeColor', 'k', 'LineStyle', ':');
    
    title('步行轨迹与拐弯区间', 'FontSize', 14);
    xlabel('X (米)'); ylabel('Y (米)'); legend('Location', 'best');
    box on;
    
    % 2. 角度变化图（标记拐弯区间）
    subplot(2,1,2);
    hold on; grid on;
    
    % 绘制角度变化曲线
    plot(t_angles, angles_deg, 'b-', 'LineWidth', 1);
    yline(90, 'r--', '90度', 'LineWidth', 1.5);
    yline(60, 'k:', '阈值下限');
    yline(120, 'k:', '阈值上限');
    
    % 高亮显示拐弯区间对应的角度变化
    for i = 1:4
        seg = turn_segments(i,:);
        % 映射到角度索引（拐弯区间在原始轨迹中的索引→角度索引）
        angle_seg = (seg(1)-1):(seg(2)-2);
        if angle_seg(1) < 1, angle_seg(1) = 1; end
        if angle_seg(end) > length(t_angles), angle_seg(end) = length(t_angles); end
        plot(t_angles(angle_seg), angles_deg(angle_seg), 'LineWidth', 3, ...
            'Color', colors(i,:), 'DisplayName', ['拐弯区间 ', num2str(i)]);
    end
    
    title('方向角变化（拐弯区间标记）', 'FontSize', 14);
    xlabel('时间 (秒)'); ylabel('角度变化 (度)'); ylim([0, 180]);
    box on;
    
    sgtitle('2×6m矩形轨迹的拐弯区间检测', 'FontSize', 16);
end