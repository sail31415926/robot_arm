function eMeetArm_workspace_IK()
%% 机械臂位置可达空间域分析（IK 扫描法，固定末端朝向）
% 在三维空间网格上对每个点调用逆运动学（IK），
% 判断在给定末端朝向约束下该点是否可达，
% 绘制三维点云及截面图，并统计可达空间尺寸。

clc; close all;
cd(fileparts(mfilename('fullpath')));

%% 1. 加载机器人模型
robot = importrobot(fullfile('eMeetArm_models', 'urdf', 'eMeetArm_models.urdf'));
robot.DataFormat = 'row';

q0 = homeConfiguration(robot);   % IK 初始猜测（零位）

%% 2. 搜索空间与末端朝向定义
% ---- 修改此处以设定目标朝向（ZYX 欧拉角，单位：度）----
% [0, 180, 0] = 末端竖直向下；[0, 0, 0] = 与基座姿态一致
targetEulerDeg = [0, 0, 90];   % 摄像头朝世界 +X 方向（零位自然朝向）
% -------------------------------------------------------

R_target = eul2rotm(deg2rad(targetEulerDeg), 'ZYX');

% 末端连杆名称
endLink = 'Link6';

% 搜索空间范围（m）与分辨率
% 0.05 m：约 14000 候选点，数分钟内完成；0.03 m 约 72000 点，耗时较长
xs = -0.9 : 0.05 : 0.9;
ys = -0.9 : 0.05 : 0.9;
zs = -0.3 : 0.05 : 1.3;

% 臂展范围预估（用于预过滤，单位 m）
reach_min = 0.08;   % 最小臂展（奇异附近）
reach_max = 1.10;   % 放宽上限，避免高 Z 区域被预过滤误删

[Xg, Yg, Zg] = ndgrid(xs, ys, zs);
gridPoints = [Xg(:), Yg(:), Zg(:)];
N_total = size(gridPoints, 1);

% ---- 优化 1：按臂展预过滤，剔除几何上不可能可达的点 ----
dist2 = sum(gridPoints.^2, 2);
prefilter = dist2 >= reach_min^2 & dist2 <= reach_max^2;
gridFiltered = gridPoints(prefilter, :);
N_filtered = size(gridFiltered, 1);

fprintf('网格总点数: %d，预过滤后: %d (减少 %.0f%%)\n', ...
        N_total, N_filtered, (1 - N_filtered/N_total)*100);
fprintf('目标朝向 ZYX(deg): [%.1f, %.1f, %.1f]\n', targetEulerDeg);

%% 3. IK 扫描 — 逐点求解逆运动学
% 权重 [朝向(3) 位置(3)]
weights = [1 1 1 1 1 1];

% 目标位姿模板：旋转部分固定，平移部分逐点填入
T_template        = eye(4);
T_template(1:3,1:3) = R_target;

maxIter   = 30;     % 迭代上限：30 次在速度和精度间平衡
errThresh = 1e-2;   % 收敛阈值：1e-2 适合可视化，1e-3 更严格但可达点更少

% ---- IK 自检：用零位正运动学位姿反求，验证 IK 基本可用 ----
ik_check = inverseKinematics('RigidBodyTree', robot);
ik_check.SolverParameters.MaxIterations = 100;
T_home = getTransform(robot, q0, endLink);
[~, info_check] = ik_check(endLink, T_home, weights, q0);
fprintf('IK 自检 PoseErrorNorm = %.5f（< 1e-3 说明 IK 正常）\n', info_check.PoseErrorNorm);

% 同时打印零位末端朝向对应的 ZYX 欧拉角，供参考
euler_home = rad2deg(rotm2eul(T_home(1:3,1:3), 'ZYX'));
fprintf('零位末端朝向 ZYX(deg): [%.1f, %.1f, %.1f]\n', euler_home);
fprintf('（可将上面数值填入 targetEulerDeg 作为有效朝向参考）\n\n');

fprintf('正在扫描 %d 个候选点（maxIter=%d, errThresh=%.0e）...\n', ...
        N_filtered, maxIter, errThresh);

ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations = maxIter;
ik.SolverParameters.MaxTime       = 0.1;   % 单点最长 0.1 s，防止个别点卡死

reachable_f = false(N_filtered, 1);
reportStep  = max(1, floor(N_filtered / 20));   % 每 5% 报告一次

tic;
for i = 1:N_filtered
    T_i = T_template;
    T_i(1:3,4) = gridFiltered(i,:)';
    [~, solInfo] = ik(endLink, T_i, weights, q0);
    reachable_f(i) = solInfo.PoseErrorNorm < errThresh;

    if mod(i, reportStep) == 0
        pct = round(i / N_filtered * 100);
        t_used = toc;
        t_left = t_used / i * (N_filtered - i);
        fprintf('  %3d%%  已用 %4.0f s  预计剩余 %4.0f s\n', pct, t_used, t_left);
    end
end
elapsed = toc;

% 把过滤后的结果映射回完整网格
reachable = false(N_total, 1);
reachable(prefilter) = reachable_f;

positions = gridPoints(reachable, :);
N = size(positions, 1);

fprintf('扫描完成，耗时 %.1f 秒\n', elapsed);
fprintf('可达点: %d / %d 候选 (%.1f%%)\n', N, N_filtered, N/N_filtered*100);

save('workspace_positions.mat', 'positions', 'targetEulerDeg');
fprintf('可达点已保存至 workspace_positions.mat\n');

%% 4. 统计信息
xRange = [min(positions(:,1)), max(positions(:,1))];
yRange = [min(positions(:,2)), max(positions(:,2))];
zRange = [min(positions(:,3)), max(positions(:,3))];
maxReach = max(sqrt(sum(positions.^2, 2)));
minReach = min(sqrt(sum(positions.^2, 2)));

fprintf('\n=== 可达空间统计 ===\n');
fprintf('X 范围: [%.3f, %.3f] m\n', xRange(1), xRange(2));
fprintf('Y 范围: [%.3f, %.3f] m\n', yRange(1), yRange(2));
fprintf('Z 范围: [%.3f, %.3f] m\n', zRange(1), zRange(2));
fprintf('最大臂展: %.3f m\n', maxReach);
fprintf('最小臂展（奇异附近）: %.3f m\n', minReach);

% 用体素网格估算工作空间体积
voxelSize = 0.01;   % 1 cm 体素
voxIdx = floor(positions / voxelSize);
nVoxels = size(unique(voxIdx, 'rows'), 1);
volume = nVoxels * voxelSize^3 * 1e6;   % 转换为 cm^3
fprintf('估算工作空间体积: %.0f cm³  (体素尺寸 %.0f mm)\n', volume, voxelSize*1000);

%% 5. 三维点云（按 Z 高度着色）
figure('Name','可达空间 - 三维点云','NumberTitle','off','Position',[50 50 900 700]);

% 点数少时全部显示且点更大，点数多时稀疏显示
if N <= 500
    idx    = 1:N;
    ptSize = 30;
else
    idx    = 1:4:N;
    ptSize = 6;
end
scatter3(positions(idx,1), positions(idx,2), positions(idx,3), ptSize, ...
         positions(idx,3), 'filled');
colormap(jet); colorbar;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('末端可达点云（%d 点，按 Z 着色）', numel(idx)));
axis equal; grid on; view(45, 25);

%% 6. XZ 截面（Y ≈ 0，侧视可达边界）
figure('Name','可达空间 - 截面图','NumberTitle','off','Position',[980 50 800 700]);

tol = 0.05;
maskXZ = abs(positions(:,2)) < tol;

subplot(1,2,1);
scatter(positions(maskXZ,1), positions(maskXZ,3), 1, 'b', 'filled');
xlabel('X (m)'); ylabel('Z (m)');
title(sprintf('XZ 截面 (|Y|<%.0f mm)', tol*1000));
axis equal; grid on;

zMid = (zRange(1) + zRange(2)) / 2;
maskXY = abs(positions(:,3) - zMid) < tol;

subplot(1,2,2);
scatter(positions(maskXY,1), positions(maskXY,2), 1, 'r', 'filled');
xlabel('X (m)'); ylabel('Y (m)');
title(sprintf('XY 截面 (Z≈%.2f m)', zMid));
axis equal; grid on;

sgtitle('可达空间截面图');

%% 7. 等值面（Isosurface / Marching Cubes）
% vRes 与 IK 网格步长一致，避免相邻点之间产生空洞碎片
gridStep = xs(2) - xs(1);   % 取 IK 搜索网格的步长
vRes = gridStep;

xVec = (xRange(1)-vRes) : vRes : (xRange(2)+vRes);
yVec = (yRange(1)-vRes) : vRes : (yRange(2)+vRes);
zVec = (zRange(1)-vRes) : vRes : (zRange(2)+vRes);

[gX, gY, gZ] = meshgrid(xVec, yVec, zVec);
occupied = false(size(gX));

xi = round((positions(:,1) - xVec(1)) / vRes) + 1;
yi = round((positions(:,2) - yVec(1)) / vRes) + 1;
zi = round((positions(:,3) - zVec(1)) / vRes) + 1;

valid = xi>=1 & xi<=numel(xVec) & yi>=1 & yi<=numel(yVec) & zi>=1 & zi<=numel(zVec);
linIdx = sub2ind(size(occupied), yi(valid), xi(valid), zi(valid));
occupied(linIdx) = true;

% 加大高斯平滑核，弥合 IK 点云间隙，减少碎片
volData = double(occupied);
sigma   = 2.5;
k       = fspecial3('gaussian', [11 11 11], sigma);
volData = convn(volData, k, 'same');

% 去除孤立碎片：只保留体素场中值最大的连续区域
% 阈值设为平滑后最大值的 20%，低于此的孤立小块被过滤掉
isoVal = max(volData(:)) * 0.20;
fv = isosurface(gX, gY, gZ, volData, isoVal);

if isempty(fv.faces)
    warning('等值面为空，请降低 isoVal 或增大搜索范围。');
    return;
end

% 去除面数少于总面数 0.5% 的碎片块（保留主体）
if size(fv.faces, 1) > 200
    % 用顶点连通性找碎片：面积极小的孤立块直接删除
    triObj  = triangulation(fv.faces, fv.vertices);
    adjMat  = vertexAttachments(triObj);
    degrees = cellfun(@numel, adjMat);
    keepV   = degrees >= 2;                          % 去掉悬挂顶点
    fv      = reducepatch(fv, sum(keepV)/size(fv.vertices,1));
end

figure('Name','可达空间 - 等值面','NumberTitle','off','Position',[50 400 900 680]);

p = patch('Vertices', fv.vertices, 'Faces', fv.faces, ...
          'FaceColor', [0.2 0.6 1.0], ...
          'EdgeColor', 'none', ...
          'FaceAlpha', 0.55);
isonormals(gX, gY, gZ, volData, p);

hold on;
plot3(0, 0, 0, 'r*', 'MarkerSize', 14, 'LineWidth', 2);
text(0.02, 0, 0, '  基座', 'Color','r', 'FontSize', 11);

lighting gouraud;
material([0.3 0.8 0.4 20]);
light('Position', [1  1  2], 'Style','infinite');
light('Position', [-1 -1 0], 'Style','infinite');

axis equal; grid on; view(40, 28);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('可达空间等值面  (体素 %.0f mm, N=%d)', vRes*1000, N));
colormap(gca, [0.2 0.6 1.0]);

%% 8. 柱坐标 r-z 包络线
r = sqrt(positions(:,1).^2 + positions(:,2).^2);

nBins  = 60;
zEdges = linspace(zRange(1), zRange(2), nBins+1);
zCenters = (zEdges(1:end-1) + zEdges(2:end)) / 2;
rMax = nan(nBins, 1);
rMin = nan(nBins, 1);

for b = 1:nBins
    mask = positions(:,3) >= zEdges(b) & positions(:,3) < zEdges(b+1);
    if sum(mask) > 5
        rMax(b) = max(r(mask));
        rMin(b) = min(r(mask));
    end
end

valid8 = ~isnan(rMax);

figure('Name','可达空间 - 柱坐标 r-z 包络','NumberTitle','off','Position',[50 50 560 560]);

fill([rMin(valid8); flipud(rMax(valid8))], ...
     [zCenters(valid8)'; flipud(zCenters(valid8)')], ...
     [0.6 0.8 1.0], 'EdgeColor','none', 'FaceAlpha', 0.5);
hold on;
plot(rMax(valid8), zCenters(valid8), 'b-',  'LineWidth', 2, 'DisplayName', '最大臂展 r_{max}');
plot(rMin(valid8), zCenters(valid8), 'b--', 'LineWidth', 1.5, 'DisplayName', '最小臂展 r_{min}');
scatter(r(1:8:end), positions(1:8:end,3), 1, [0.7 0.7 0.7], 'filled');

xlabel('水平半径 r = \surd(x²+y²)  (m)');
ylabel('高度 z  (m)');
title('可达空间柱坐标 r-z 包络');
legend('可达区域','最大半径','最小半径','Location','best');
grid on; xlim([0, max(r)*1.05]); ylim([zRange(1)-0.02, zRange(2)+0.02]);
plot(0, 0, 'r*', 'MarkerSize', 12, 'LineWidth', 2);

%% 9. 多高度 XY 截面
nLayers = 6;
zLayers = linspace(zRange(1) + 0.05, zRange(2) - 0.05, nLayers);
tol2    = (zRange(2) - zRange(1)) / nLayers / 2;

colors = jet(nLayers);
figure('Name','可达空间 - 多层 XY 截面','NumberTitle','off','Position',[640 50 900 800]);

for k = 1:nLayers
    subplot(2, 3, k);
    maskZ = abs(positions(:,3) - zLayers(k)) < tol2;
    if sum(maskZ) < 10
        title(sprintf('z ≈ %.2f m\n（点数不足）', zLayers(k)));
        continue;
    end
    scatter(positions(maskZ,1), positions(maskZ,2), 2, ...
            colors(k,:), 'filled');
    hold on;
    plot(0, 0, 'k+', 'MarkerSize', 10, 'LineWidth', 2);
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('z ≈ %.2f m  (%d 点)', zLayers(k), sum(maskZ)));
    axis equal; grid on;
    xlim([xRange(1)-0.02, xRange(2)+0.02]);
    ylim([yRange(1)-0.02, yRange(2)+0.02]);
end

sgtitle(sprintf('可达空间多层 XY 截面（层厚 ±%.0f mm）', tol2*1000));

end
