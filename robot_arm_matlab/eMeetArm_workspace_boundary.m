function eMeetArm_workspace_boundary()
%% eMeetArm 可达空间边界描述（Alpha Shape 网格）
%
% 输出：
%   workspace_boundary.mat  —— 边界网格 (vertices/faces) + alphaShape 对象
%   workspace_boundary.stl  —— 可导入 CAD/仿真软件的 STL 网格
%
% 上层规划器使用：
%   load('workspace_boundary.mat', 'shp');
%   ok = inShape(shp, px, py, pz);          % 点是否在可达空间内
%   d  = nearestNeighbor(shp, px, py, pz);  % 到边界最近点索引

clc; close all;
cd(fileparts(mfilename('fullpath')));

%% 1. 加载可达点云
if ~exist('workspace_positions.mat', 'file')
    error('请先运行 eMeetArm_workspace_IK.m 生成 workspace_positions.mat');
end
load('workspace_positions.mat', 'positions', 'targetEulerDeg');
fprintf('加载 %d 个可达点，朝向 ZYX(deg): [%.1f %.1f %.1f]\n', ...
        size(positions,1), targetEulerDeg);

%% 2. 构建 Alpha Shape
% alpha 值控制边界紧致程度：
%   越小 → 越贴合点云，可能出现内部空洞
%   越大 → 越平滑，趋向凸包
% 默认按 IK 网格步长的 2 倍设定，自动适配分辨率
gridStep = 0.05;   % 与 IK 脚本保持一致
alphaVal = gridStep * 2.5;

shp = alphaShape(positions(:,1), positions(:,2), positions(:,3), alphaVal);
shp.HoleThreshold = 1e6;   % 填充所有内部空洞（设为极大值）

fprintf('Alpha Shape 构建完成：alpha=%.3f m，体积=%.4f m³\n', ...
        shp.Alpha, volume(shp));

%% 3. 提取边界网格（顶点 + 三角面）
[faces, verts] = boundaryFacets(shp);
fprintf('边界网格：%d 个顶点，%d 个三角面\n', size(verts,1), size(faces,1));

%% 4. 可视化
figure('Name','可达空间边界网格','NumberTitle','off','Position',[100 100 1000 750]);

% 边界曲面（半透明蓝）
trisurf(faces, verts(:,1), verts(:,2), verts(:,3), ...
        'FaceColor', [0.2 0.6 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.45);
hold on;

% 可达点云（稀疏显示）
idx = 1:3:size(positions,1);
scatter3(positions(idx,1), positions(idx,2), positions(idx,3), ...
         3, [0.1 0.3 0.8], 'filled', 'DisplayName', '可达点');

% 基座
plot3(0,0,0,'r*','MarkerSize',14,'LineWidth',2,'DisplayName','基座');

lighting gouraud;
light('Position',[1 1 2],'Style','infinite');
light('Position',[-1 -1 0],'Style','infinite');
material([0.3 0.8 0.3 20]);

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('可达空间边界网格  alpha=%.3f m，体积=%.4f m³', alphaVal, volume(shp)));
axis equal; grid on; view(45,25);

%% 5. XZ / XY 截面轮廓
figure('Name','边界截面轮廓','NumberTitle','off','Position',[1100 100 700 600]);

tol = 0.06;

subplot(1,2,1);
mask = abs(positions(:,2)) < tol;
scatter(positions(mask,1), positions(mask,3), 4, [0.2 0.5 0.9], 'filled');
xlabel('X (m)'); ylabel('Z (m)');
title(sprintf('XZ 截面 |Y|<%.0fmm', tol*1000));
axis equal; grid on;

subplot(1,2,2);
zMid = (min(positions(:,3)) + max(positions(:,3))) / 2;
mask = abs(positions(:,3) - zMid) < tol;
scatter(positions(mask,1), positions(mask,2), 4, [0.2 0.5 0.9], 'filled');
xlabel('X (m)'); ylabel('Y (m)');
title(sprintf('XY 截面 Z≈%.2fm', zMid));
axis equal; grid on;

sgtitle('可达空间截面轮廓');

%% 6. 保存
save('workspace_boundary.mat', 'shp', 'faces', 'verts', 'targetEulerDeg', 'alphaVal');
fprintf('边界数据已保存至 workspace_boundary.mat\n');

% 导出 STL（可导入 SolidWorks / RViz / 仿真环境）
stlwrite(triangulation(faces, verts), 'workspace_boundary.stl');
fprintf('STL 文件已导出至 workspace_boundary.stl\n');

%% 7. 打印规划器使用方法
fprintf('\n=== 上层规划器调用示例 ===\n');
fprintf('load(''workspace_boundary.mat'', ''shp'');\n\n');
fprintf('%% 检查目标点是否在可达空间内\n');
fprintf('ok = inShape(shp, px, py, pz);\n\n');
fprintf('%% 批量检查路径点（N×3 矩阵）\n');
fprintf('ok_all = inShape(shp, path(:,1), path(:,2), path(:,3));\n');
fprintf('valid  = all(ok_all);  %% 整条路径是否可行\n\n');
fprintf('%% 找到最近的可达点（当目标不可达时）\n');
fprintf('[~,idx] = min(sum((verts - [px py pz]).^2, 2));\n');
fprintf('nearest = verts(idx, :);\n');

end
