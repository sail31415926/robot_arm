# eMeetArm MATLAB 分析指南

基于 Robotics System Toolbox，当前已有 URDF 模型可直接使用。

---

## 已有脚本

| 文件 | 功能 |
| --- | --- |
| `eMeetArm_workspace.m` | 蒙特卡洛法求无姿态约束的最大可达空间 |
| `eMeetArm_workspace_IK.m` | IK 扫描法求固定末端朝向下的可达空间 |
| `eMeetArm_joint_control.m` | 滑块交互控制，3D 实时显示关节位姿 |

---

## 可扩展的分析方向

### 1. 可操作性分析（Manipulability）

评估每个构型的运动灵巧度，接近奇异时值趋近于 0。

```matlab
robot = importrobot(fullfile('eMeetArm_models','urdf','eMeetArm_models.urdf'));
robot.DataFormat = 'row';
q = homeConfiguration(robot);

% 计算单个构型的可操作性
m = manipulability(robot, q, 'Link6');

% 扫描关节空间，绘制可操作性分布热图
% （在 eMeetArm_workspace.m 的采样循环基础上添加）
```

**用途**：找出设计中灵巧度低的区域，优化连杆比例。

---

### 2. 奇异性分析

雅可比矩阵行列式接近零的构型为奇异构型，末端运动能力退化。

```matlab
J = geometricJacobian(robot, q, 'Link6');   % 6×6 几何雅可比
sigma = svd(J);                              % 奇异值分解
condNum = max(sigma) / min(sigma);           % 条件数，越大越接近奇异
manipIdx = prod(sigma);                      % 可操作性指标
```

**用途**：设计时规避奇异区域，规划路径时绕开奇异点。

---

### 3. 关节空间轨迹规划

在两个构型之间生成平滑的关节轨迹（梯形速度曲线）。

```matlab
robot = importrobot(fullfile('eMeetArm_models','urdf','eMeetArm_models.urdf'));
robot.DataFormat = 'row';

q_start = homeConfiguration(robot);
q_end   = [0.5, 1.0, -1.5, 0.8, 0.3, -0.5];   % 目标构型

% 生成轨迹：100 个时间步，总时长 3 秒
[q_traj, qd_traj, qdd_traj, t] = trapveltraj([q_start; q_end]', 100, ...
    'EndTime', 3);

% 动画播放
figure;
ax = axes;
for i = 1:size(q_traj, 2)
    show(robot, q_traj(:,i)', 'Parent', ax, ...
         'FastUpdate', true, 'PreservePlot', false);
    drawnow limitrate;
end
```

**用途**：验证设计的运动是否平滑，关节速度/加速度是否在限制内。

---

### 4. 笛卡尔直线轨迹规划

末端从 A 点沿直线运动到 B 点（需 IK 逐点求解）。

```matlab
robot = importrobot(fullfile('eMeetArm_models','urdf','eMeetArm_models.urdf'));
robot.DataFormat = 'row';
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.25 0.25 0.25 1 1 1];   % 位置权重高于姿态
q0 = homeConfiguration(robot);

% 定义起末端点（齐次变换矩阵）
T_start = getTransform(robot, q0, 'Link6');
T_end   = T_start;
T_end(1:3, 4) = T_start(1:3, 4) + [0.1; 0; -0.05];   % 沿 X+0.1, Z-0.05 移动

nSteps = 50;
q_traj = zeros(nSteps, 6);
q_prev = q0;

for i = 1:nSteps
    alpha = (i-1) / (nSteps-1);
    T_interp = T_start;
    T_interp(1:3, 4) = (1-alpha)*T_start(1:3,4) + alpha*T_end(1:3,4);
    [q_sol, ~] = ik('Link6', T_interp, weights, q_prev);
    q_traj(i,:) = q_sol;
    q_prev = q_sol;
end
```

**用途**：验证末端能否沿特定方向做直线运动（如抓取下压路径）。

---

### 5. 逆动力学 —— 关节力矩计算

给定运动轨迹，计算各关节所需力矩，用于电机选型。

```matlab
robot = importrobot(fullfile('eMeetArm_models','urdf','eMeetArm_models.urdf'));
robot.DataFormat = 'row';

% 示例：匀速运动（加速度为零）
q    = homeConfiguration(robot);
qd   = zeros(1, 6);    % 关节速度
qdd  = zeros(1, 6);    % 关节加速度

% 计算重力 + 惯性 + 科氏力矩（需要 URDF 中有质量/惯量信息）
tau = inverseDynamics(robot, q, qd, qdd);

fprintf('各关节力矩 (N·m):\n');
for i = 1:6
    fprintf('  Joint%d: %.3f N·m\n', i, tau(i));
end
```

**注意**：URDF 中已包含每个连杆的质量和惯量数据，结果可直接用于电机选型参考。

**用途**：校验减速比设计，确认各关节电机额定力矩是否满足要求。

---

### 6. 工作空间 + 灵巧度联合分析

在可达空间分析基础上，叠加可操作性热图，找出"既可达又灵巧"的高质量工作区域。

```matlab
% 在 eMeetArm_workspace.m 的采样循环中，额外记录每个点的可操作性
manip_vals = zeros(N, 1);
for i = 1:N
    % ... 原有 FK 计算 ...
    manip_vals(i) = manipulability(robot, qFull(i,:), 'Link6');
end

% 按可操作性着色绘制点云
scatter3(positions(:,1), positions(:,2), positions(:,3), ...
         1, manip_vals, 'filled');
colormap(hot); colorbar;
title('可达空间（可操作性着色，越亮越灵巧）');
```

---

## 关节参数速查（来自 URDF）

| 关节 | 下限 (rad) | 上限 (rad) | 说明 |
| --- | --- | --- | --- |
| Joint1 | -3.1 | 3.1 | 腰转 |
| Joint2 | -0.8 | 3.14 | 大臂 |
| Joint3 | -3.14 | 0.05 | 小臂 |
| Joint4 | -3.1 | 3.1 | 腕摆 |
| Joint5 | -0.7854 | 0.7854 | 腕转 |
| Joint6 | -1.5 | 0.5 | 末端旋转 |

## 末端坐标系

末端连杆名称：`Link6`（所有脚本统一使用此名称）
