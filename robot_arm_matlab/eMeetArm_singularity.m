function eMeetArm_singularity()
%% eMeetArm 奇异性分析
% 在关节空间随机采样，对每个构型计算几何雅可比矩阵，
% 通过可操作性指标（μ）和条件数（κ）定量评估奇异程度，
% 三维可视化标注高危奇异区域。

clc; close all;
cd(fileparts(mfilename('fullpath')));

%% 1. 加载机器人模型
robot = importrobot(fullfile('eMeetArm_models', 'urdf', 'eMeetArm_models.urdf'));
robot.DataFormat = 'row';
q0 = homeConfiguration(robot);

%% 2. 采样参数
N = 50000;

lower = [-3.1,  -0.8,  -3.14,  -3.1,   -0.7854,  -1.5];
upper = [ 3.1,   3.14,  0.05,   3.1,    0.7854,   0.5];

endLink = 'Link6';

% 接近奇异的判定阈值（可操作性 μ 低于此值视为高危）
singThresh = 0.01;

%% 3. 采样 + 雅可比计算
fprintf('正在采样并计算雅可比矩阵（N=%d），请稍候...\n', N);

qSamples = rand(N, 6) .* (upper - lower) + lower;
qFull    = repmat(q0, N, 1);

positions  = zeros(N, 3);
manip_vals = zeros(N, 1);   % 可操作性指标 μ = ∏σᵢ
cond_vals  = zeros(N, 1);   % 条件数 κ = σ_max / σ_min
sigma_mins = zeros(N, 1);   % 最小奇异值（最直接的奇异度量）

tic;
for i = 1:N
    qFull(i, 1:6) = qSamples(i, :);

    T = getTransform(robot, qFull(i,:), endLink);
    positions(i,:) = T(1:3, 4)';

    J     = geometricJacobian(robot, qFull(i,:), endLink);
    sigma = svd(J);

    manip_vals(i) = prod(sigma);
    cond_vals(i)  = max(sigma) / (min(sigma) + 1e-12);
    sigma_mins(i) = min(sigma);
end
elapsed = toc;
fprintf('完成，耗时 %.1f 秒\n', elapsed);

%% 4. 统计汇总
maskSing = manip_vals < singThresh;
nSing    = sum(maskSing);

fprintf('\n=== 奇异性统计 ===\n');
fprintf('可操作性 μ：最小 %.5f，最大 %.4f，中位数 %.4f\n', ...
        min(manip_vals), max(manip_vals), median(manip_vals));
fprintf('条件数   κ：最小 %.1f，最大 %.1f\n', min(cond_vals), max(cond_vals));
fprintf('接近奇异 (μ < %.3f)：%d / %d 个构型 (%.1f%%)\n', ...
        singThresh, nSing, N, nSing/N*100);

[~, worstIdx] = min(manip_vals);
fprintf('最奇异构型 (μ=%.5f) 关节角 (rad):\n  ', manip_vals(worstIdx));
fprintf('%.3f  ', qFull(worstIdx, 1:6));
fprintf('\n');

%% 5. 三维点云 —— 按可操作性着色
% 截断到 95 百分位，防止极大值压缩颜色动态范围
mClip = min(manip_vals, prctile(manip_vals, 95));

figure('Name','奇异性 - 可操作性点云','NumberTitle','off','Position',[50 50 900 700]);

idx = 1:4:N;
scatter3(positions(idx,1), positions(idx,2), positions(idx,3), ...
         2, mClip(idx), 'filled');
colormap(hot); cb = colorbar;
cb.Label.String = '可操作性 μ（越亮越灵巧）';
hold on;

% 奇异点红色高亮
scatter3(positions(maskSing,1), positions(maskSing,2), positions(maskSing,3), ...
         6, 'c', 'filled', 'DisplayName', sprintf('高危奇异 (μ<%.3f)', singThresh));

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('末端可达空间 - 可操作性着色  (暗红=奇异, 亮黄=灵巧, N=%d)', numel(idx)));
legend('Location','best');
axis equal; grid on; view(45, 25);

%% 6. XZ 截面奇异热图
figure('Name','奇异性 - XZ 截面','NumberTitle','off','Position',[980 50 780 600]);

tol = 0.04;
maskXZ = abs(positions(:,2)) < tol;

scatter(positions(maskXZ,1), positions(maskXZ,3), 3, mClip(maskXZ), 'filled');
colormap(hot); colorbar;
hold on;
scatter(positions(maskSing & maskXZ,1), positions(maskSing & maskXZ,3), ...
        10, 'c', 'filled');

xlabel('X (m)'); ylabel('Z (m)');
title(sprintf('XZ 截面奇异分布 (|Y|<%.0f mm)', tol*1000));
axis equal; grid on;

%% 7. 统计分布直方图
figure('Name','奇异性 - 统计分布','NumberTitle','off','Position',[50 400 900 550]);

subplot(1,2,1);
histogram(manip_vals, 120, 'FaceColor',[0.2 0.5 0.8], 'EdgeColor','none');
xline(singThresh, 'r--', 'LineWidth', 2, 'Label', ...
      sprintf('阈值 %.3f\n(%.1f%%)', singThresh, nSing/N*100), ...
      'LabelVerticalAlignment','bottom');
xlabel('可操作性 μ'); ylabel('频次');
title('可操作性分布'); grid on;

subplot(1,2,2);
histogram(log10(cond_vals + 1), 100, 'FaceColor',[0.8 0.4 0.2], 'EdgeColor','none');
xlabel('log_{10}(κ + 1)'); ylabel('频次');
title('条件数分布（值越大越病态）'); grid on;

sgtitle('eMeetArm 奇异性统计分布');

%% 8. 六关节对可操作性的影响（箱线图）
figure('Name','奇异性 - 关节贡献分析','NumberTitle','off','Position',[980 400 900 550]);

% 将关节角归一化到 [0,1]，分 5 档，看各档内可操作性中位数
nBins  = 8;
colors = lines(6);
jointNames = {'J1腰转','J2大臂','J3小臂','J4腕摆','J5腕转','J6末端'};

for j = 1:6
    subplot(2, 3, j);
    qj = qSamples(:, j);
    edges = linspace(lower(j), upper(j), nBins+1);
    binMed = zeros(nBins, 1);
    binCnt = zeros(nBins, 1);
    for b = 1:nBins
        mask = qj >= edges(b) & qj < edges(b+1);
        if sum(mask) > 0
            binMed(b) = median(manip_vals(mask));
            binCnt(b) = sum(mask);
        end
    end
    centers = (edges(1:end-1) + edges(2:end)) / 2;
    bar(centers, binMed, 'FaceColor', colors(j,:), 'EdgeColor','none');
    xlabel('关节角 (rad)'); ylabel('中位数 μ');
    title(jointNames{j}); grid on;
end

sgtitle('各关节角度区间对可操作性的影响（中位数）');

%% 9. 最奇异构型可视化
figure('Name','奇异性 - 最奇异构型','NumberTitle','off','Position',[50 50 650 550]);
show(robot, qFull(worstIdx,:), 'Visuals','on', 'Frames','on');
title(sprintf('最奇异构型  μ = %.5f\nJ = [%.2f  %.2f  %.2f  %.2f  %.2f  %.2f] rad', ...
      manip_vals(worstIdx), qFull(worstIdx,1:6)));
view(45, 20); axis equal; grid on;

end
