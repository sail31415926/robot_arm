function eMeetArm_workspace_ellipsoid()
%% eMeetArm 可达空间椭球拟合
% 对 IK 扫描得到的可达点云拟合最小包围椭球，
% 输出中心、矩阵、半轴等参数供上层规划器使用。
%
% 上层规划器约束公式：
%   (p - c)' * P * (p - c) <= 1
%   p : 3×1 末端目标位置 (m)
%   c : 椭球中心 (3×1)
%   P : 椭球约束矩阵 (3×3，= inv(A))
%
% 参数保存在 workspace_ellipsoid.mat，直接 load 即可使用。

clc; close all;
cd(fileparts(mfilename('fullpath')));

%% 1. 加载可达点云
if ~exist('workspace_positions.mat', 'file')
    error('未找到 workspace_positions.mat，请先运行 eMeetArm_workspace_IK.m');
end
load('workspace_positions.mat', 'positions', 'targetEulerDeg');
N = size(positions, 1);
fprintf('加载 %d 个可达点，目标朝向 ZYX(deg): [%.1f %.1f %.1f]\n', ...
        N, targetEulerDeg);

%% 2. 椭球拟合（协方差法 + 外扩余量）
c = mean(positions)';        % 椭球中心（3×1）
C = cov(positions);          % 协方差矩阵（3×3）

% 计算每个点到中心的 Mahalanobis 距离，取最大值作为缩放因子
diffs    = positions - c';
mah_dist = sqrt(sum((diffs / C) .* diffs, 2));
k        = max(mah_dist) * 1.05;   % 额外 5% 余量，确保完全包含所有点

A = C * k^2;   % 椭球矩阵：(p-c)' * inv(A) * (p-c) <= 1
P = inv(A);    % 规划器直接使用的约束矩阵

% 特征值分解求半轴长度和方向
[V, D] = eig(A);
semi_axes = sqrt(diag(D));   % 三个半轴长度（m）
axes_dirs = V;               % 列向量 = 对应轴方向

%% 3. 统计输出
fprintf('\n=== 椭球参数 ===\n');
fprintf('中心 c (m):  [%.4f, %.4f, %.4f]\n', c);
fprintf('半轴长度 (m): a=%.4f  b=%.4f  c=%.4f\n', sort(semi_axes,'descend'));
fprintf('覆盖率验证: ');
inside = sum(sum((diffs / A) .* diffs, 2) <= 1);
fprintf('%d / %d 点在椭球内 (%.1f%%)\n', inside, N, inside/N*100);

fprintf('\n=== 规划器使用方法 ===\n');
fprintf('load(''workspace_ellipsoid.mat'', ''c'', ''P'');\n');
fprintf('ok = (p - c)'' * P * (p - c) <= 1;\n\n');

%% 4. 保存参数
save('workspace_ellipsoid.mat', 'c', 'A', 'P', 'semi_axes', 'axes_dirs', ...
     'targetEulerDeg', 'k');
fprintf('参数已保存至 workspace_ellipsoid.mat\n');

%% 5. 可视化：实际点云 vs 椭球
figure('Name','可达空间椭球拟合','NumberTitle','off','Position',[100 100 1000 750]);

% 实际点云（蓝色）
scatter3(positions(:,1), positions(:,2), positions(:,3), ...
         6, [0.4 0.6 0.9], 'filled', 'DisplayName','可达点云');
hold on;

% 椭球曲面（半透明红色）
[sx, sy, sz] = sphere(60);
pts_sphere = [sx(:), sy(:), sz(:)]';      % 3×N 单位球面点

% 将单位球变换到椭球：p = c + A^(1/2) * pts_sphere
A_sqrt = V * diag(semi_axes) * V';        % A^(1/2)
pts_ellip = A_sqrt * pts_sphere + c;

Ex = reshape(pts_ellip(1,:), size(sx));
Ey = reshape(pts_ellip(2,:), size(sy));
Ez = reshape(pts_ellip(3,:), size(sz));

surf(Ex, Ey, Ez, ...
     'FaceColor', [1.0 0.4 0.3], ...
     'EdgeColor', 'none', ...
     'FaceAlpha', 0.25, ...
     'DisplayName', sprintf('拟合椭球 (a=%.2f b=%.2f c=%.2f m)', ...
                            sort(semi_axes,'descend')));

% 基座
plot3(0, 0, 0, 'k*', 'MarkerSize', 14, 'LineWidth', 2, 'DisplayName','基座');

% 椭球中心
plot3(c(1), c(2), c(3), 'r+', 'MarkerSize', 12, 'LineWidth', 2, ...
      'DisplayName', sprintf('椭球中心 (%.2f,%.2f,%.2f)', c));

% 三条半轴（有向线段）
colors = {'r','g','b'};
labels = {'轴1','轴2','轴3'};
for i = 1:3
    ep = c + axes_dirs(:,i) * semi_axes(i);
    plot3([c(1) ep(1)], [c(2) ep(2)], [c(3) ep(3)], ...
          colors{i}, 'LineWidth', 2, 'DisplayName', ...
          sprintf('%s %.2fm', labels{i}, semi_axes(i)));
end

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('可达空间椭球拟合  (覆盖率 %.1f%%)', inside/N*100));
legend('Location','best'); axis equal; grid on; view(45, 25);

%% 6. XZ 截面对比（侧视）
figure('Name','椭球拟合 - XZ 截面','NumberTitle','off','Position',[1100 100 600 600]);

tol = 0.06;
maskXZ = abs(positions(:,2)) < tol;
scatter(positions(maskXZ,1), positions(maskXZ,3), 6, [0.4 0.6 0.9], 'filled');
hold on;

% 椭球在 Y≈0 处的截面（椭圆）
theta = linspace(0, 2*pi, 300);
% 参数化椭球 XZ 截面（令 Y=c(2)，在椭球方程中求 X-Z 椭圆）
% 近似：用椭球主轴在 XZ 平面的投影
xy_pts = [cos(theta); zeros(1,300); sin(theta)];  % Y=0 平面上的单位圆
ellip_xz = A_sqrt * xy_pts + c;
plot(ellip_xz(1,:), ellip_xz(3,:), 'r-', 'LineWidth', 2);

xlabel('X (m)'); ylabel('Z (m)');
title('XZ 截面：实际点云 vs 椭球轮廓');
legend('可达点 (|Y|<60mm)', '椭球截面', 'Location','best');
axis equal; grid on;

end
