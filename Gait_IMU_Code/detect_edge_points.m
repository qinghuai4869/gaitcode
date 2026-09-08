% 主函数：输入轨迹和时间，输出四条边上的各两个点（共8个点）
function edge_points = detect_edge_points(traj, t)
    rng(0); % 固定随机种子
    
    % 提取x、y坐标
    x = traj(1, :);
    y = traj(2, :);
    
    % 输入检查
    if length(x) ~= length(t) || length(y) ~= length(t)
        error('轨迹坐标与时间长度不匹配！');
    end
    if length(x) < 20
        error('轨迹点数不足，请提供至少20个点！');
    end
    
    % 检测四条边上的点
    [edge_points, x_smooth, y_smooth] = find_edge_points(x, y, t);
    
    % 可视化结果
%     visualize_edge_points(x, y, t, edge_points, x_smooth, y_smooth);
end


function [edge_points, x_smooth, y_smooth] = find_edge_points(x, y, t)
    % 1. 平滑轨迹（保留边的线性特征）
    window = 5;
    x_smooth = movmean(x, window, 'Endpoints', 'fill');
    y_smooth = movmean(y, window, 'Endpoints', 'fill');
    
    % 2. 定义矩形四条边的几何约束（基于2×6m矩形，W=-2）
    % 边1：底边（从左下(0,0)到右下(6,0)）→ y≈0，x从0到6（排除拐角附近）
    % 边2：右边（从右下(6,0)到右上(6,-2)）→ x≈6，y从0到-2（排除拐角附近）
    % 边3：顶边（从右上(6,-2)到左上(0,-2)）→ y≈-2，x从6到0（排除拐角附近）
    % 边4：左边（从左上(0,-2)到左下(0,0)）→ x≈0，y从-2到0（排除拐角附近）
    edge_constraints = struct();
    % 边1：底边（y在-0.5到0.5之间，x在0.5到5.5之间，避开拐角）
    edge_constraints(1).x_min = 0.5;
    edge_constraints(1).x_max = 5.5;
    edge_constraints(1).y_min = -0.5;
    edge_constraints(1).y_max = 0.5;
    edge_constraints(1).name = '底边';
    
    % 边2：右边（x在5.5到6.5之间，y在-1.5到-0.5之间，避开拐角）
    edge_constraints(2).x_min = 5.5;
    edge_constraints(2).x_max = 6.5;
    edge_constraints(2).y_min = -1.5;
    edge_constraints(2).y_max = -0.5;
    edge_constraints(2).name = '右边';
    
    % 边3：顶边（y在-2.5到-1.5之间，x在0.5到5.5之间，避开拐角）
    edge_constraints(3).x_min = 0.5;
    edge_constraints(3).x_max = 5.5;
    edge_constraints(3).y_min = -2.5;
    edge_constraints(3).y_max = -1.5;
    edge_constraints(3).name = '顶边';
    
    % 边4：左边（x在-0.5到0.5之间，y在-1.5到-0.5之间，避开拐角）
    edge_constraints(4).x_min = -0.5;
    edge_constraints(4).x_max = 0.5;
    edge_constraints(4).y_min = -1.5;
    edge_constraints(4).y_max = -0.5;
    edge_constraints(4).name = '左边';
    
    % 3. 为每条边筛选符合条件的点（排除噪声点）
    edge_points = cell(4, 1); % 存储4条边的点（每条边2个）
    for i = 1:4
        % 获取当前边的约束
        ec = edge_constraints(i);
        % 筛选在边区域内的点
        in_edge = (x_smooth >= ec.x_min) & (x_smooth <= ec.x_max) & ...
                  (y_smooth >= ec.y_min) & (y_smooth <= ec.y_max);
        edge_candidates = find(in_edge); % 符合条件的索引
        
        % 若候选点不足，适当放宽范围（增强鲁棒性）
        if length(edge_candidates) < 5
            warning('边%s候选点不足，放宽筛选范围', ec.name);
            in_edge = (x_smooth >= ec.x_min-0.3) & (x_smooth <= ec.x_max+0.3) & ...
                      (y_smooth >= ec.y_min-0.3) & (y_smooth <= ec.y_max+0.3);
            edge_candidates = find(in_edge);
        end
        
        % 4. 从候选点中选2个点（按时间顺序，均匀分布在边上）
        if length(edge_candidates) >= 2
            % 按时间排序（确保沿轨迹方向）
            edge_candidates = sort(edge_candidates);
            % 计算时间间隔，选前半段和后半段各1个点（避免集中在一处）
            mid_idx = floor(length(edge_candidates)/2);
            selected_idx = [edge_candidates(1), edge_candidates(mid_idx)];
        else
            % 极端情况：候选点不足2个，直接取所有候选点（至少1个）
            selected_idx = edge_candidates;
        end
        
        % 5. 存储点的坐标和时间（原始轨迹值，非平滑值）
        edge_points{i} = [x(selected_idx)', y(selected_idx)', t(selected_idx)'];
    end
    
    % 转换为矩阵（8×3：前2行底边，3-4行右边，5-6行顶边，7-8行左边）
    edge_points = cell2mat(edge_points);
end


function visualize_edge_points(x, y, t, edge_points, x_smooth, y_smooth)
    % 可视化四条边上的点
    figure('Position', [100, 100, 1200, 700]);
    hold on; axis equal; grid on; grid minor;
    
    % 绘制原始轨迹和平滑轨迹
    plot(x, y, 'b-', 'LineWidth', 1, 'DisplayName', '原始轨迹');
    plot(x_smooth, y_smooth, 'k--', 'LineWidth', 1, 'DisplayName', '平滑轨迹');
    
    % 标记四条边上的点（用不同颜色区分）
    colors = lines(4); % 4种颜色对应4条边
    labels = {'底边点', '右边点', '顶边点', '左边点'};
    for i = 1:4
        % 每条边的2个点（索引：边1→1-2，边2→3-4，以此类推）
        idx = (i-1)*2 + 1 : i*2;
        plot(edge_points(idx,1), edge_points(idx,2), 'o', ...
            'MarkerSize', 9, 'MarkerFaceColor', colors(i,:), ...
            'MarkerEdgeColor', 'k', 'LineWidth', 1, 'DisplayName', labels{i});
        % 标记点编号（边号+点序号，如1-1表示底边第一个点）
        for j = 1:2
            text(edge_points(idx(j),1)+0.2, edge_points(idx(j),2)+0.2, ...
                [num2str(i) '-', num2str(j)], 'Color', 'k', 'FontSize', 10);
        end
    end
    
    % 绘制四条边的区域范围（灰色虚线框）
    plot([0,6,6,0,0], [0,0,-2,-2,0], 'g--', 'LineWidth', 1.5, 'DisplayName', '矩形边界');
    
    % 图表标注
    title('矩形四条边上的特征点检测结果', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X 坐标 (米)', 'FontSize', 12);
    ylabel('Y 坐标 (米)', 'FontSize', 12);
    legend('Location', 'best', 'FontSize', 10);
    box on;
    axis([-1,7,-3,1]); % 调整坐标轴范围，完整显示矩形
end