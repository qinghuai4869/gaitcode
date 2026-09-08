function corners = detect_rectangle_corners(traj,t)
    rng(0); % 固定随机种子，确保结果可复现
    
    % 1. 生成更陡峭的转弯轨迹
    x=traj(1,:);
    y=traj(2,:);
        
    % 2. 结合几何约束的拐角检测
    [corners, angles_deg, t_angles] = geometry_constrained_detection(x, y, t);
    
    % 3. 增强可视化（显示中间过程）
%     visualize_with_details(x, y, t, corners, angles_deg, t_angles);
end


function [corners, angles_deg, t_angles] = geometry_constrained_detection(x, y, t)
    % 结合几何约束的拐角检测：已知矩形大致位置，强制从四个顶点区域找拐角
    
    % 1. 轻度平滑（保留转弯特征）
    window = 5;  % 小窗口，避免模糊转弯
    x_smooth = movmean(x, window);
    y_smooth = movmean(y, window);
    
    % 2. 计算方向角变化（更灵敏的角度计算）
    dx = diff(x_smooth);
    dy = diff(y_smooth);
    dir_angle = atan2(dy, dx);  % 方向角（弧度）
    d_angle = diff(dir_angle);  % 方向角变化
    
    % 处理角度周期性（将变化映射到[-π, π]，直角转弯约为±π/2）
    d_angle = mod(d_angle + pi, 2*pi) - pi;
    angle_change = abs(d_angle);  % 转弯角度大小（取绝对值）
    angles_deg = rad2deg(angle_change);  % 转换为角度
    t_angles = t(2:end-1);  % 角度对应的时间点
    
    % 3. 初步筛选：角度变化在60~120度之间的候选点
    % 原来是80~95
    candidate_idx = find(angles_deg >= 60 & angles_deg <= 120);
    if isempty(candidate_idx)
        warning('无候选拐角点，放宽阈值');
        candidate_idx = 1:length(angles_deg);  % 极端情况：全部视为候选
    end
    
    % 4. 几何约束：矩形四个顶点的大致区域（已知2×6m）
    % 换成3个顶点；
    L = 6; W = -2;
    target_areas = [
        L-0.2, L+0.2, 0-0.2, 0+0.2;  % 右下拐角区域 (6,0)附近
        L-0.5, L+0.5, W-0.5, W+0.5;  % 右上拐角区域 (6,-2)附近
        0-0.2, 0+0.2, W-0.2, W+0.2;  % 左上拐角区域 (0,-2)附近
%         0-0.5, 0+0.5, 0-0.5, 0+0.5  % 左下拐角区域 (0,0)附近
    ];  % [xmin, xmax, ymin, ymax]
    
    % 5. 从候选点中，为每个目标区域选1个最可能的拐角
    corners = [];
    for i = 1:3  % 四个目标区域
        area = target_areas(i,:);
        xmin = area(1); xmax = area(2);
        ymin = area(3); ymax = area(4);
        
        % 找出候选点中位于当前区域的点
        in_area = false(size(candidate_idx));
        for j = 1:length(candidate_idx)
            pos_idx = candidate_idx(j) + 1;  % 映射回原始轨迹索引
            x_pos = x_smooth(pos_idx);
            y_pos = y_smooth(pos_idx);
            in_area(j) = (x_pos >= xmin) && (x_pos <= xmax) && ...
                         (y_pos >= ymin) && (y_pos <= ymax);
        end
        area_candidates = candidate_idx(in_area);
        
        % 如果区域内有候选点，选角度最接近90度的；否则选区域内最近的点
        if ~isempty(area_candidates)
            % 按角度接近90度排序
            [~, sort_idx] = sort(abs(angles_deg(area_candidates) - 90));
            best_idx = area_candidates(sort_idx(1));
        else
            % 从整个轨迹中找区域内最近的点
            all_pos_idx = 1:length(x_smooth);
            in_area_all = (x_smooth >= xmin) & (x_smooth <= xmax) & ...
                          (y_smooth >= ymin) & (y_smooth <= ymax);
            area_all = all_pos_idx(in_area_all);
            if ~isempty(area_all)
                best_idx = area_all(1);  % 取区域内第一个点
            else
                % 极端情况：区域内无点，用最近点
                [~, best_idx] = min((x_smooth - (xmin+xmax)/2).^2 + ...
                                    (y_smooth - (ymin+ymax)/2).^2);
            end
            best_idx = best_idx - 1;  % 映射回角度索引
        end
        
        % 记录拐角点（原始坐标+时间）
        pos_idx = best_idx + 1;  % 角度索引→轨迹索引
        corners = [corners; x(pos_idx), y(pos_idx), t(pos_idx)];
    end
    
    % 6. 按时间排序拐角点
    [~, sort_idx] = sort(corners(:,3));
    corners = corners(sort_idx,:);
end

function visualize_with_details(x, y, t, corners, angles_deg, t_angles)
    % 显示详细中间结果，帮助调试
    figure('Position', [100, 100, 1200, 800]);
    
    % 1. 轨迹图（带目标区域）
    subplot(2,1,1);
    hold on; axis equal; grid on;
    plot(x, y, 'b-', 'LineWidth', 1, 'DisplayName', '轨迹');
    plot(x(1), y(1), 'go', 'MarkerSize', 8, 'DisplayName', '起点');
    plot(x(end), y(end), 'mo', 'MarkerSize', 8, 'DisplayName', '终点');
    
    % 标记四个目标区域（矩形拐角的大致位置）
    L = 6; W = -2;
    rectangle('Position', [L-1, 0-1, 2, 2], 'EdgeColor', 'k', 'LineStyle', ':');
    rectangle('Position', [L-1, W-1, 2, 2], 'EdgeColor', 'k', 'LineStyle', ':');
    rectangle('Position', [0-1, W-1, 2, 2], 'EdgeColor', 'k', 'LineStyle', ':');
%     rectangle('Position', [0-1, 0-1, 2, 2], 'EdgeColor', 'k', 'LineStyle', ':');
    
    % 绘制检测到的拐角点
    plot(corners(:,1), corners(:,2), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', '拐角点');
    for i = 1:3
        text(corners(i,1)+0.2, corners(i,2)+0.2, num2str(i), ...
            'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
    end
    
    title('轨迹与拐角点（带目标区域）', 'FontSize', 14);
    xlabel('X (米)'); ylabel('Y (米)'); legend('Location', 'best');
    box on;
    
    % 2. 角度变化图（标记拐角点对应的角度）
    subplot(2,1,2);
    hold on; grid on;
    plot(t_angles, angles_deg, 'b-', 'LineWidth', 1, 'DisplayName', '角度变化');
    yline(90, 'r--', '90度', 'LineWidth', 1.5);
    yline(45, 'k:', '阈值下限');
    yline(90, 'k:', '阈值上限');
    
    % 标记拐角点对应的角度
    for i = 1:3
        [~, idx] = min(abs(t_angles - corners(i,3)));
        plot(t_angles(idx), angles_deg(idx), 'ro', 'MarkerSize', 8);
        text(t_angles(idx), angles_deg(idx)+5, num2str(i), 'Color', 'r');
    end
    
    title('方向角变化（拐角点对应位置）', 'FontSize', 14);
    xlabel('时间 (秒)'); ylabel('角度变化 (度)'); ylim([0, 180]);
    box on;
    
    sgtitle('2×6m矩形轨迹拐角点检测（带几何约束）', 'FontSize', 16);
end