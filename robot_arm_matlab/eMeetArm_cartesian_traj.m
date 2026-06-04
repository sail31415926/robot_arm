function eMeetArm_cartesian_traj()
%% eMeetArm 笛卡尔直线轨迹规划
% 末端从起点沿直线运动到终点，全程保持末端朝向不变。
% 流程：插值笛卡尔路径点 → 逐点 IK → 动画播放 → 轨迹曲线

clc; close all;
cd(fileparts(mfilename('fullpath')));

%% 1. 加载机器人模型
robot = importrobot(fullfile('eMeetArm_models', 'urdf', 'eMeetArm_models.urdf'));
robot.DataFormat = 'row';
endLink = 'Link6';

ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations = 60;
weights = [1 1 1 1 1 1];   % 朝向和位置同等权重

%% 2. 定义起末位姿
q_start = homeConfiguration(robot);
T_start = getTransform(robot, q_start, endLink);

% ---- 修改此处设定目标位置偏移（相对起点，单位 m）----
delta = [0.15; 0.0; -0.10];   % X +15cm，Z -10cm
% ----------------------------------------------------

T_end = T_start;
T_end(1:3, 4) = T_start(1:3, 4) + delta;

p_start = T_start(1:3, 4)';
p_end   = T_end(1:3, 4)';

fprintf('起点位置: [%.3f  %.3f  %.3f] m\n', p_start);
fprintf('终点位置: [%.3f  %.3f  %.3f] m\n', p_end);
fprintf('直线距离: %.3f m\n', norm(delta));

%% 3. 笛卡尔路径插值（梯形速度曲线）
nSteps   = 80;    % 路径点数
duration = 3.0;   % 运动总时长 (s)

% 梯形速度曲线：加速段 20%，匀速段 60%，减速段 20%
t_norm = trapveltraj([0, 1], nSteps, 'EndTime', 1)';   % 0→1 归一化时间

% 位置：沿 delta 方向线性插值
pos_traj = p_start + t_norm .* (p_end - p_start);   % nSteps×3

% 朝向保持不变（全程固定为起点朝向）
R_fixed = T_start(1:3, 1:3);

%% 4. 逐点 IK 求解关节角
fprintf('正在求解 %d 个路径点的 IK...\n', nSteps);

q_traj  = zeros(nSteps, length(q_start));
ik_ok   = true(nSteps, 1);
q_prev  = q_start;

tic;
for i = 1:nSteps
    T_i = eye(4);
    T_i(1:3, 1:3) = R_fixed;
    T_i(1:3, 4)   = pos_traj(i, :)';

    [q_sol, info] = ik(endLink, T_i, weights, q_prev);
    q_traj(i, :)  = q_sol;
    ik_ok(i)      = info.PoseErrorNorm < 5e-3;
    q_prev        = q_sol;
end
elapsed = toc;

nFail = sum(~ik_ok);
fprintf('IK 完成，耗时 %.2f 秒', elapsed);
if nFail > 0
    fprintf('，%d 个点未收敛（标红）', nFail);
end
fprintf('\n');

%% 5. 动画播放
t_vec = linspace(0, duration, nSteps);

fig1 = figure('Name','笛卡尔直线轨迹动画','NumberTitle','off', ...
              'Position',[50 50 900 700]);
axR = axes('Parent', fig1);

show(robot, q_traj(1,:), 'Parent', axR, 'Visuals','on', 'Frames','off');
hold(axR, 'on');

% 预画末端轨迹线
plot3(axR, pos_traj(:,1), pos_traj(:,2), pos_traj(:,3), ...
      'b--', 'LineWidth', 1.5, 'DisplayName', '规划路径');

% 标注起止点
plot3(axR, p_start(1), p_start(2), p_start(3), ...
      'go', 'MarkerSize', 10, 'MarkerFaceColor','g', 'DisplayName','起点');
plot3(axR, p_end(1), p_end(2), p_end(3), ...
      'rs', 'MarkerSize', 10, 'MarkerFaceColor','r', 'DisplayName','终点');

axis(axR,'equal'); grid(axR,'on'); view(axR, 45, 20);
xlabel(axR,'X (m)'); ylabel(axR,'Y (m)'); zlabel(axR,'Z (m)');
legend(axR,'Location','best');

% 实际末端轨迹（逐点更新）
actual_pos = zeros(nSteps, 3);
hTrace = plot3(axR, NaN, NaN, NaN, 'r-', 'LineWidth', 2, 'DisplayName','实际轨迹');

fprintf('开始播放动画...\n');
for i = 1:nSteps
    show(robot, q_traj(i,:), 'Parent', axR, ...
         'Visuals','on', 'Frames','off', ...
         'FastUpdate', true, 'PreservePlot', false);

    T_actual     = getTransform(robot, q_traj(i,:), endLink);
    actual_pos(i,:) = T_actual(1:3, 4)';
    set(hTrace, 'XData', actual_pos(1:i,1), ...
                'YData', actual_pos(1:i,2), ...
                'ZData', actual_pos(1:i,3));

    % 颜色标注 IK 失败点
    if ~ik_ok(i)
        plot3(axR, actual_pos(i,1), actual_pos(i,2), actual_pos(i,3), ...
              'rx', 'MarkerSize', 8, 'LineWidth', 2);
    end

    title(axR, sprintf('t = %.2f s  末端 (%.3f, %.3f, %.3f) m', ...
          t_vec(i), actual_pos(i,:)));
    drawnow limitrate;
    pause(duration / nSteps * 0.8);
end

%% 6. 关节角曲线
fig2 = figure('Name','关节角轨迹','NumberTitle','off','Position',[980 50 900 650]);
jointNames = {'J1 腰转','J2 大臂','J3 小臂','J4 腕摆','J5 腕转','J6 末端'};
colors = lines(6);

for j = 1:6
    subplot(2, 3, j);
    plot(t_vec, rad2deg(q_traj(:, j)), 'Color', colors(j,:), 'LineWidth', 1.8);
    hold on;
    % 关节限位虚线
    lower = [-3.1,  -0.8,  -3.14,  -3.1,   -0.7854,  -1.5];
    upper = [ 3.1,   3.14,  0.05,   3.1,    0.7854,   0.5];
    yline(rad2deg(lower(j)), 'r--', 'LineWidth', 1, 'Alpha', 0.6);
    yline(rad2deg(upper(j)), 'r--', 'LineWidth', 1, 'Alpha', 0.6);
    xlabel('时间 (s)'); ylabel('角度 (deg)');
    title(jointNames{j}); grid on;
end
sgtitle('各关节角度随时间变化（红虚线 = 关节限位）');

%% 7. 末端位置误差（规划 vs 实际）
pos_err = sqrt(sum((actual_pos - pos_traj).^2, 2)) * 1000;   % mm

figure('Name','末端位置误差','NumberTitle','off','Position',[50 420 700 350]);
plot(t_vec, pos_err, 'b-', 'LineWidth', 1.8);
xlabel('时间 (s)'); ylabel('位置误差 (mm)');
title(sprintf('IK 跟踪误差（最大 %.2f mm，均值 %.2f mm）', ...
              max(pos_err), mean(pos_err)));
grid on;
yline(5, 'r--', '5 mm 阈值', 'LineWidth', 1.2);

end
