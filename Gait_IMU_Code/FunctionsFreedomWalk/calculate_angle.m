function theta = calculate_angle(a, b)
% 计算两个向量的夹角
%
% 输入：
%   a: 第一个向量
%   b: 第二个向量
%
% 输出：
%   theta: 两个向量之间的夹角（以度为单位）

% 计算点积
dot_product = dot(a, b);

% 计算模长
norm_a = norm(a);
norm_b = norm(b);

% 计算夹角的余弦值
cos_theta = dot_product / (norm_a * norm_b);

% 计算夹角（以弧度为单位）
theta = acos(cos_theta);

% 将弧度转换为度
theta = rad2deg(theta);

end