function eMeetArm_workspace()
%% 机械臂位置可达空间域分析（蒙特卡洛法）
% 在 joint1~joint6 的关节限位内随机采样，正运动学求末端位置，
% 绘制三维点云及截面图，并统计可达空间尺寸。
% 不考虑末端姿态；本机械臂为 6 自由度（Joint1~Joint6），无夹爪关节。

clc; close all;
cd(fileparts(mfilename('fullpath')));

%% 1. 加载机器人模型
% 统一使用 robot_arm_description 包中的 URDF（与仿真/实机一致），
% 该 URDF 位于工作空间同级目录 ../robot_arm_description/urdf/ 下。
% 说明：本脚本仅做正运动学，不渲染网格，importrobot 若提示找不到
%       package:// 网格文件（可忽略），不影响可达域计算。
urdfPath = fullfile('..', 'robot_arm_description', 'urdf', 'eMeetArm_models.urdf');
robot = importrobot(urdfPath);
robot.DataFormat = 'row';

%% 2. 采样参数
N = 80000;   % 采样点数（越大越准确，耗时也越长）

% Joint1~Joint6 的关节限位（来自 robot_arm_description 的 URDF）
% 2026-07-28 实机重标定：J2 新零点 = 旧 +0.181 rad，J3 = 旧 -0.176 rad，限位随刻度平移；
% 同日 J3 行程按新零点收紧为 [-2.5, 0]
lower = [-2.618, -0.981, -2.5,  -3.1,  -0.7854, -1.5];
upper = [ 2.618,  2.959,  0.02,  3.1,   0.7854,  0.5];

% 末端连杆名称（用于 getTransform）
endLink = 'Link6';

%% 3. 蒙特卡洛采样 + 正运动学
fprintf('正在采样 %d 个构型，请稍候...\n', N);

% 在限位范围内均匀随机采样 joint1~6，每行一个构型
qSamples = rand(N, 6) .* (upper - lower) + lower;

% 预分配末端位置矩阵
positions = zeros(N, 3);

tic;
for i = 1:N
    T = getTransform(robot, qSamples(i, :), endLink);
    positions(i, :) = T(1:3, 4)';
end
elapsed = toc;
fprintf('采样完成，耗时 %.1f 秒\n', elapsed);

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

% 每隔若干点绘制，避免过密（取 1/4 的点显示）
idx = 1:4:N;
scatter3(positions(idx,1), positions(idx,2), positions(idx,3), 1, ...
         positions(idx,3), 'filled');
colormap(jet); colorbar;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('Piper 末端可达点云（%d 点，按 Z 着色）', numel(idx)));
axis equal; grid on; view(45, 25);

%% 6. XZ 截面（Y ≈ 0，侧视可达边界）
figure('Name','可达空间 - 截面图','NumberTitle','off','Position',[980 50 800 700]);

% 选取 |Y| < 0.05m 附近的点显示侧视截面
tol = 0.05;
maskXZ = abs(positions(:,2)) < tol;

subplot(1,2,1);
scatter(positions(maskXZ,1), positions(maskXZ,3), 1, 'b', 'filled');
xlabel('X (m)'); ylabel('Z (m)');
title(sprintf('XZ 截面 (|Y|<%.0f mm)', tol*1000));
axis equal; grid on;

% XY 截面（Z ≈ 某高度，俯视）
zMid = (zRange(1) + zRange(2)) / 2;
maskXY = abs(positions(:,3) - zMid) < tol;

subplot(1,2,2);
scatter(positions(maskXY,1), positions(maskXY,2), 1, 'r', 'filled');
xlabel('X (m)'); ylabel('Y (m)');
title(sprintf('XY 截面 (Z≈%.2f m)', zMid));
axis equal; grid on;

sgtitle('Piper 可达空间截面图');

%% 7. 等值面（Isosurface / Marching Cubes）
% 将采样点投影到三维体素网格，再用 isosurface 提取外壳曲面

% 体素分辨率（越小越精细，但内存和耗时增加）
vRes = 0.030;   % 1.5 cm

% 构建覆盖所有采样点的网格坐标轴（各方向留一个体素余量）
xVec = (xRange(1)-vRes) : vRes : (xRange(2)+vRes);
yVec = (yRange(1)-vRes) : vRes : (yRange(2)+vRes);
zVec = (zRange(1)-vRes) : vRes : (zRange(2)+vRes);

% 初始化体素占用矩阵（0 = 不可达，1 = 可达）
[gX, gY, gZ] = meshgrid(xVec, yVec, zVec);   % 注意 meshgrid 行=Y, 列=X, 层=Z
occupied = false(size(gX));

% 将每个采样点映射到体素索引并标记为已占用
xi = round((positions(:,1) - xVec(1)) / vRes) + 1;
yi = round((positions(:,2) - yVec(1)) / vRes) + 1;
zi = round((positions(:,3) - zVec(1)) / vRes) + 1;

% 过滤越界索引（理论上不应越界，保险起见）
valid = xi>=1 & xi<=numel(xVec) & yi>=1 & yi<=numel(yVec) & zi>=1 & zi<=numel(zVec);
linIdx = sub2ind(size(occupied), yi(valid), xi(valid), zi(valid));
occupied(linIdx) = true;

% 高斯平滑体素场，使等值面更平滑（避免阶梯状锯齿）
volData = double(occupied);
sigma   = 1.2;   % 平滑核标准差（体素单位）
k       = fspecial3('gaussian', [5 5 5], sigma);
volData = convn(volData, k, 'same');

% 提取等值面（阈值 0.15：可调，越小边界越外扩）
isoVal = 0.15;
fv = isosurface(gX, gY, gZ, volData, isoVal);

% 若等值面为空（采样点太少或阈值过高），给出提示并跳过
if isempty(fv.faces)
    warning('等值面为空，请降低 isoVal 或增大 N。');
    return;
end

figure('Name','可达空间 - 等值面','NumberTitle','off','Position',[50 400 900 680]);

% 创建 patch，再对 patch 对象调用 isonormals（自动处理边界顶点，不会越界）
p = patch('Vertices', fv.vertices, 'Faces', fv.faces, ...
          'FaceColor', [0.2 0.6 1.0], ...   % 蓝色外壳
          'EdgeColor', 'none', ...
          'FaceAlpha', 0.55);               % 半透明，可看到内部结构
isonormals(gX, gY, gZ, volData, p);        % 直接对 patch 计算法线，避免越界错误

% 叠加机械臂基座原点
hold on;
plot3(0, 0, 0, 'r*', 'MarkerSize', 14, 'LineWidth', 2);
text(0.02, 0, 0, '  基座', 'Color','r', 'FontSize', 11);

% 光照与材质
lighting gouraud;
material([0.3 0.8 0.4 20]);    % 环境光/漫反射/高光/光泽度
light('Position', [1  1  2], 'Style','infinite');
light('Position', [-1 -1 0], 'Style','infinite');

axis equal; grid on; view(40, 28);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('Piper 可达空间等值面  (体素 %.0f mm, N=%d)', vRes*1000, N));
colormap(gca, [0.2 0.6 1.0]);

%% 8. 柱坐标 r-z 包络线
% r = sqrt(x²+y²) 是末端到基座轴线的水平距离
% 将 z 轴等分成若干区间，统计每个区间内 r 的最大/最小值，形成包络带

r = sqrt(positions(:,1).^2 + positions(:,2).^2);   % 水平半径

nBins  = 60;   % z 方向分层数
zEdges = linspace(zRange(1), zRange(2), nBins+1);
zCenters = (zEdges(1:end-1) + zEdges(2:end)) / 2;
rMax = nan(nBins, 1);
rMin = nan(nBins, 1);

for b = 1:nBins
    mask = positions(:,3) >= zEdges(b) & positions(:,3) < zEdges(b+1);
    if sum(mask) > 5   % 至少 5 个点才统计，避免稀疏层噪声
        rMax(b) = max(r(mask));
        rMin(b) = min(r(mask));
    end
end

valid8 = ~isnan(rMax);

figure('Name','可达空间 - 柱坐标 r-z 包络','NumberTitle','off','Position',[50 50 560 560]);

% 填充包络带（rMin ~ rMax 之间为可达区域）
fill([rMin(valid8); flipud(rMax(valid8))], ...
     [zCenters(valid8)'; flipud(zCenters(valid8)')], ...
     [0.6 0.8 1.0], 'EdgeColor','none', 'FaceAlpha', 0.5);
hold on;
plot(rMax(valid8), zCenters(valid8), 'b-',  'LineWidth', 2, 'DisplayName', '最大臂展 r_{max}');
plot(rMin(valid8), zCenters(valid8), 'b--', 'LineWidth', 1.5, 'DisplayName', '最小臂展 r_{min}');

% 散点（稀疏显示，避免遮挡包络线）
scatter(r(1:8:end), positions(1:8:end,3), 1, [0.7 0.7 0.7], 'filled');

xlabel('水平半径 r = \surd(x²+y²)  (m)');
ylabel('高度 z  (m)');
title('Piper 可达空间柱坐标 r-z 包络');
legend('可达区域','最大半径','最小半径','Location','best');
grid on; xlim([0, max(r)*1.05]); ylim([zRange(1)-0.02, zRange(2)+0.02]);
plot(0, 0, 'r*', 'MarkerSize', 12, 'LineWidth', 2);   % 基座投影

%% 9. 多高度 XY 截面
% 在 z 轴等间距选 6 个高度层，展示俯视可达覆盖范围
nLayers = 6;
zLayers = linspace(zRange(1) + 0.05, zRange(2) - 0.05, nLayers);
tol2    = (zRange(2) - zRange(1)) / nLayers / 2;   % 每层厚度为层间距的一半

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
    plot(0, 0, 'k+', 'MarkerSize', 10, 'LineWidth', 2);   % 基座投影
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('z ≈ %.2f m  (%d 点)', zLayers(k), sum(maskZ)));
    axis equal; grid on;
    xlim([xRange(1)-0.02, xRange(2)+0.02]);
    ylim([yRange(1)-0.02, yRange(2)+0.02]);
end

sgtitle(sprintf(' 可达空间多层 XY 截面（层厚 ±%.0f mm）', tol2*1000));

end
