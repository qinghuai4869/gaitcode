% 辅助函数：检测拐点
function turning_points = find_turning_points(traj, thresh)
        N = size(traj, 1);
        turning_points = [];
        
        if N < 3
            return;
        end
        
        vectors = diff(traj);
        for ki = 2:(N-1)
            v1 = vectors(ki-1, :);
            v2 = vectors(ki, :);
            
            % 归一化向量
            if norm(v1) > 0, v1 = v1/norm(v1); end
            if norm(v2) > 0, v2 = v2/norm(v2); end
            
            angle = atan2d(abs(det([v1; v2])), dot(v1, v2));
            
            if angle > thresh
                turning_points = [turning_points; traj(ki, :)];
            end
        end
  end