function [turning_segments, turning_centers] = find_turning_segments(trajectory, curvature_threshold, min_segment_length)
    % trajectory: N x 2矩阵，每行是一个坐标点[x, y]
    % curvature_threshold: 曲率阈值（默认0.15）
    % min_segment_length: 最小拐弯段长度（默认5个点）
    
    if nargin < 2
        curvature_threshold = 0.15;
    end
    if nargin < 3
        min_segment_length = 5;
    end
    
    % 1. 计算稳健曲率（不平滑轨迹）
    curvature = calculate_robust_curvature(trajectory, 3);
    
    % 2. 标记高曲率点
    is_high_curvature = curvature > curvature_threshold;
    
    % 3. 合并相邻点为连续段
    [segment_start, segment_end] = find_continuous_segments(is_high_curvature, min_segment_length);
    
    % 4. 提取拐弯段信息
    turning_segments = cell(length(segment_start), 1);
    turning_centers = zeros(length(segment_start), 2);
    
    for i = 1:length(segment_start)
        seg_idx = segment_start(i):segment_end(i);
        turning_segments{i} = trajectory(seg_idx, :);
        
        % 计算段中心点（曲率最大的点）
        [~, max_idx] = max(curvature(seg_idx));
        turning_centers(i, :) = trajectory(seg_idx(max_idx), :);
    end
end

function [segment_start, segment_end] = find_continuous_segments(is_high, min_length)
    % 找到连续的满足条件的段
    diff_is_high = diff([0; is_high; 0]);
    segment_start = find(diff_is_high == 1);
    segment_end = find(diff_is_high == -1) - 1;
    
    % 过滤掉过短的段
    valid = (segment_end - segment_start + 1) >= min_length;
    segment_start = segment_start(valid);
    segment_end = segment_end(valid);
end

