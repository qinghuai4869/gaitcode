function curvature = calculate_robust_curvature(traj, window)
    % 多尺度差分计算曲率（不平滑轨迹）
    N = size(traj, 1);
    curvature = zeros(N, 1);
    
    for i = (window+1):(N-window)
        % 局部多项式拟合
        t = (1:2*window+1)';
        px = polyfit(t, traj(i-window:i+window, 1), 2);
        py = polyfit(t, traj(i-window:i+window, 2), 2);
        
        % 中心点导数
        t_center = window + 1;
        dx = 2*px(1)*t_center + px(2);
        dy = 2*py(1)*t_center + py(2);
        ddx = 2*px(1);
        ddy = 2*py(1);
        
        curvature(i) = abs(dx*ddy - dy*ddx) / (dx^2 + dy^2 + eps)^(3/2);
    end
end