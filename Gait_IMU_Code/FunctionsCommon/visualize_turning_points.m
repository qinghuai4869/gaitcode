%%画图拐点
function visualize_turning_points(trajectory, angle_threshold)
    % trajectory: N x 2矩阵，每行是一个坐标点[x, y]
    % angle_threshold: 拐弯的角度阈值（单位：度）
    
    if nargin < 2
        angle_threshold = 30; % 默认角度阈值
    end
    
    % 检测拐点
    turning_points = find_turning_points(trajectory, angle_threshold);
    
    % 创建图形窗口
    figure('Color', 'white', 'Position', [100 100 800 600]);
    ax = gca;
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    
    % 绘制原始轨迹（带方向箭头）
    quiver(ax, trajectory(1:end-1,1), trajectory(1:end-1,2), ...
           diff(trajectory(:,1)), diff(trajectory(:,2)), ...
           0, 'b-', 'LineWidth', 1.5, 'MaxHeadSize', 0.5, ...
           'DisplayName', '轨迹方向');
    
    % 绘制轨迹点
    plot(ax, trajectory(:,1), trajectory(:,2), 'bo-', ...
         'MarkerSize', 6, 'LineWidth', 1, 'MarkerFaceColor', 'b', ...
         'DisplayName', '轨迹点');
    
    % 标记拐点
    if ~isempty(turning_points)
        plot(ax, turning_points(:,1), turning_points(:,2), 'rs', ...
             'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'r', ...
             'DisplayName', sprintf('拐点(阈值%d°)', angle_threshold));
        
        % 为每个拐点添加编号
        for ki = 1:size(turning_points,1)
            text(ax, turning_points(ki,1)+0.1, turning_points(ki,2)+0.1, ...
                 num2str(ki), 'Color', 'r', 'FontWeight', 'bold');
        end
    end
    
    % 添加图例和标题
    legend(ax, 'Location', 'best', 'AutoUpdate', 'off');
    title(ax, sprintf('轨迹拐点检测 (阈值: %d°)', angle_threshold));
    xlabel(ax, 'X坐标');
    ylabel(ax, 'Y坐标');
    
    % 显示检测到的拐点坐标
    fprintf('检测到的拐点坐标（阈值%d°）：\n', angle_threshold);
    disp(turning_points);
 end